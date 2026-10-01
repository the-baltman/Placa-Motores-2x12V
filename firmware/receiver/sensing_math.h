// sensing_math.h - pure conversions, hysteresis and soft current limiter (tested on the PC).
#pragma once
#include <stdint.h>

#include "config.h"

namespace sens_math {

inline float vbat_from_adc_mv(float adc_mv) { return adc_mv / 1000.0f / cfg::VBAT_DIV_K; }

struct CsEstimate {
  bool valid;      // amps can be trusted (duty high enough, ADC not saturated)
  bool is_fault;   // IS pin above the driver-fault level (BTS7960 flags a fault with ~4.5 mA)
  float amps;      // 0..CS_I_CLAMP_A
};

// The IS output only flows while the high-side FET is on, and the board RC filter averages it,
// so what the ADC sees is D*I*0.0588 V (+ GND offset). Divide by D to recover I.
inline CsEstimate cs_estimate(float adc_mv, float offset_mv, float duty) {
  CsEstimate e{false, false, 0.0f};
  if (adc_mv >= cfg::CS_IS_FAULT_MV) {
    e.is_fault = true;
    return e;
  }
  if (duty < cfg::CS_MIN_DUTY_VALID) return e;
  float net_mv = adc_mv - offset_mv;
  if (net_mv < 0.0f) net_mv = 0.0f;
  float amps = (net_mv / 1000.0f) / (cfg::CS_VOLT_PER_AMP * duty);
  if (amps > cfg::CS_I_CLAMP_A) amps = cfg::CS_I_CLAMP_A;
  e.valid = true;
  e.amps = amps;
  return e;
}

// Schmitt threshold with optional time debounce on entry. "Low" variant: active when the value
// stays below `enter` for debounce_ms, released above `exit`.
struct HystLow {
  bool active = false;
  uint32_t acc_ms = 0;
  void update(float v, float enter, float exit, uint32_t dt_ms, uint32_t debounce_ms) {
    if (active) {
      if (v > exit) active = false;
      acc_ms = 0;
    } else if (v < enter) {
      acc_ms += dt_ms;
      if (acc_ms >= debounce_ms) { active = true; acc_ms = 0; }
    } else {
      acc_ms = 0;
    }
  }
};

// "High" variant: active above `enter`, released below `exit`.
struct HystHigh {
  bool active = false;
  uint32_t acc_ms = 0;
  void update(float v, float enter, float exit, uint32_t dt_ms, uint32_t debounce_ms) {
    if (active) {
      if (v < exit) active = false;
      acc_ms = 0;
    } else if (v > enter) {
      acc_ms += dt_ms;
      if (acc_ms >= debounce_ms) { active = true; acc_ms = 0; }
    } else {
      acc_ms = 0;
    }
  }
};

// Per-channel soft limiter. Above I_SOFT_LIMIT_A for I_SOFT_TIME_MS the command cap ramps down;
// above I_HARD_LIMIT_A for I_HARD_TIME_MS the channel trips (latched until reset()).
// It holds its state on invalid estimates: no estimate is better than a wrong one.
struct CurrentLimiter {
  int16_t cap_permille = cfg::CMD_MAX_PERMILLE;
  uint32_t over_soft_ms = 0;
  uint32_t over_hard_ms = 0;
  bool tripped = false;

  void reset() { *this = CurrentLimiter(); }

  void step(bool valid, float amps, uint32_t dt_ms) {
    if (!valid) return;
    if (amps > cfg::I_HARD_LIMIT_A) {
      over_hard_ms += dt_ms;
      if (over_hard_ms >= cfg::I_HARD_TIME_MS) tripped = true;
    } else {
      over_hard_ms = 0;
    }
    if (amps > cfg::I_SOFT_LIMIT_A) {
      over_soft_ms += dt_ms;
      if (over_soft_ms >= cfg::I_SOFT_TIME_MS) {
        int32_t c = (int32_t)cap_permille - cfg::DERATE_STEP_PERMILLE;
        if (c < cfg::DERATE_CAP_MIN_PERMILLE) c = cfg::DERATE_CAP_MIN_PERMILLE;
        cap_permille = (int16_t)c;
      }
    } else if (amps < cfg::I_SOFT_RELEASE_A) {
      over_soft_ms = 0;
      int32_t c = (int32_t)cap_permille + cfg::DERATE_RECOVER_PERMILLE;
      if (c > cfg::CMD_MAX_PERMILLE) c = cfg::CMD_MAX_PERMILLE;
      cap_permille = (int16_t)c;
    }
  }
};

}  // namespace sens_math
