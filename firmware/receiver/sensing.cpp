#include "sensing.h"

#include <Arduino.h>
#include <esp_timer.h>

#include "config.h"
#include "sensing_math.h"

namespace {

constexpr uint8_t SLOW_OVERSAMPLE = 16;      // VBAT / NTC samples per read
constexpr uint8_t SENS_FAIL_CYCLES = 3;      // consecutive implausible slow reads before sens_ok drops
constexpr uint32_t WARN_DEBOUNCE_MS = 200;
constexpr uint32_t OT_DEBOUNCE_MS = 200;

const uint8_t CS_PIN[2] = {cfg::PIN_CS_M1, cfg::PIN_CS_M2};

SensSnapshot g_s;
float g_offset_mv[2] = {0.0f, 0.0f};
float g_i_filt[2] = {0.0f, 0.0f};
sens_math::CurrentLimiter g_lim[2];
sens_math::HystLow g_warn, g_cut, g_ntc_hot;
sens_math::HystHigh g_ov;
uint8_t g_bad_vbat = 0, g_bad_ntc = 0;
bool g_slow_turn_ntc = false;
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

// Mean of n calibrated samples. Returns false if any sample reads 0 (Arduino reports 0 on a failed read).
bool read_mean_mv(uint8_t pin, uint8_t n, bool reject_zero, float& out) {
  uint32_t acc = 0;
  for (uint8_t i = 0; i < n; ++i) {
    const uint32_t mv = analogReadMilliVolts(pin);
    if (reject_zero && mv == 0) return false;
    acc += mv;
  }
  out = (float)acc / (float)n;
  return true;
}

bool vbat_plausible(float v) { return v >= cfg::VBAT_PLAUSIBLE_MIN_V && v <= cfg::VBAT_PLAUSIBLE_MAX_V; }
bool ntc_plausible(float mv) {
  return !cfg::NTC_FAULT_ENABLED || (mv >= cfg::NTC_PLAUSIBLE_MIN_MV && mv <= cfg::NTC_PLAUSIBLE_MAX_MV);
}

}  // namespace

bool sensing_init() {
  analogReadResolution(12);
  analogSetPinAttenuation(cfg::PIN_CS_M1, ADC_11db);
  analogSetPinAttenuation(cfg::PIN_CS_M2, ADC_11db);
  analogSetPinAttenuation(cfg::PIN_VBAT_SENSE, ADC_11db);
  analogSetPinAttenuation(cfg::PIN_NTC_MAX, ADC_11db);

  g_s = SensSnapshot();
  for (uint8_t ch = 0; ch < 2; ++ch) {
    g_lim[ch].reset();
    g_s.cap_permille[ch] = cfg::CMD_MAX_PERMILLE;
  }
  g_warn = sens_math::HystLow();
  g_cut = sens_math::HystLow();
  g_ntc_hot = sens_math::HystLow();
  g_ov = sens_math::HystHigh();

  float vbat_mv = 0, ntc_mv = 0;
  const bool ok_v = read_mean_mv(cfg::PIN_VBAT_SENSE, SLOW_OVERSAMPLE, true, vbat_mv);
  const bool ok_n = read_mean_mv(cfg::PIN_NTC_MAX, SLOW_OVERSAMPLE, true, ntc_mv);
  g_s.vbat_v = sens_math::vbat_from_adc_mv(vbat_mv);
  g_s.ntc_mv = ntc_mv;
  const bool v_good = ok_v && vbat_plausible(g_s.vbat_v);
  const bool n_good = ok_n && ntc_plausible(ntc_mv);
  // A failed first read starts the fail counters at the limit, so sens_ok stays false until a good read.
  g_bad_vbat = v_good ? 0 : SENS_FAIL_CYCLES;
  g_bad_ntc = n_good ? 0 : SENS_FAIL_CYCLES;
  g_slow_turn_ntc = false;
  g_s.sens_ok = v_good && n_good;
  return g_s.sens_ok;
}

bool sensing_calibrate_zero() {
  float acc[2] = {0, 0};
  for (uint16_t i = 0; i < cfg::CS_CAL_SAMPLES; ++i) {
    for (uint8_t ch = 0; ch < 2; ++ch) acc[ch] += (float)analogReadMilliVolts(CS_PIN[ch]);
    delay(1);  // setup-time only (before the control task exists); spreads samples over ~64 ms
  }
  bool ok = true;
  for (uint8_t ch = 0; ch < 2; ++ch) {
    g_offset_mv[ch] = acc[ch] / (float)cfg::CS_CAL_SAMPLES;
    if (g_offset_mv[ch] > cfg::CS_OFFSET_MAX_MV) ok = false;
  }
  if (!ok) { g_offset_mv[0] = g_offset_mv[1] = 0.0f; }
  g_s.cal_ok = ok;
  return ok;
}

void sensing_update(uint32_t dt_ms, const float duty[2]) {
  const int64_t t0 = esp_timer_get_time();
  SensSnapshot s = g_s;

  for (uint8_t ch = 0; ch < 2; ++ch) {
    float mv = 0;
    read_mean_mv(CS_PIN[ch], cfg::CS_OVERSAMPLE, false, mv);
    s.cs_mv[ch] = mv;
    s.cs_high[ch] = mv >= cfg::CS_FLT_CHANNEL_MARGIN_MV;
    const sens_math::CsEstimate e = sens_math::cs_estimate(mv, g_offset_mv[ch], duty[ch]);
    s.cs_is_fault[ch] = e.is_fault;
    s.i_valid[ch] = e.valid;
    if (e.valid) {
      g_i_filt[ch] += cfg::CS_IIR_ALPHA * (e.amps - g_i_filt[ch]);
    } else {
      g_i_filt[ch] = 0.0f;
    }
    s.i_amps[ch] = g_i_filt[ch];
    g_lim[ch].step(e.valid, g_i_filt[ch], dt_ms);
    s.cap_permille[ch] = g_lim[ch].cap_permille;
    s.trip[ch] = g_lim[ch].tripped;
  }

  // Slow channels alternate cycle by cycle: each is refreshed every 2*dt_ms.
  const uint32_t slow_dt = 2 * dt_ms;
  if (!g_slow_turn_ntc) {
    float mv = 0;
    if (read_mean_mv(cfg::PIN_VBAT_SENSE, SLOW_OVERSAMPLE, true, mv) && vbat_plausible(sens_math::vbat_from_adc_mv(mv))) {
      s.vbat_v = sens_math::vbat_from_adc_mv(mv);
      g_bad_vbat = 0;
      g_warn.update(s.vbat_v, cfg::VBAT_WARN_V, cfg::VBAT_WARN_V + cfg::VBAT_HYST_V, slow_dt, WARN_DEBOUNCE_MS);
      g_cut.update(s.vbat_v, cfg::VBAT_CUT_V, cfg::VBAT_CUT_V + cfg::VBAT_HYST_V, slow_dt, cfg::VBAT_CUT_DEBOUNCE_MS);
      g_ov.update(s.vbat_v, cfg::VBAT_OV_V, cfg::VBAT_OV_RELEASE_V, slow_dt, 0);
    } else if (g_bad_vbat < 255) {
      ++g_bad_vbat;
    }
  } else {
    float mv = 0;
    if (read_mean_mv(cfg::PIN_NTC_MAX, SLOW_OVERSAMPLE, true, mv) && ntc_plausible(mv)) {
      s.ntc_mv = mv;
      g_bad_ntc = 0;
      g_ntc_hot.update(mv, cfg::NTC_CUT_MV, cfg::NTC_RELEASE_MV, slow_dt, OT_DEBOUNCE_MS);
    } else if (g_bad_ntc < 255) {
      ++g_bad_ntc;
    }
  }
  g_slow_turn_ntc = !g_slow_turn_ntc;

  // Fail safe: an implausible battery or NTC reading for several reads drops sens_ok (disarms).
  // bad counters tick once per 2 cycles, so SENS_FAIL_CYCLES reads = 60 ms.
  s.sens_ok = g_bad_vbat < SENS_FAIL_CYCLES && g_bad_ntc < SENS_FAIL_CYCLES;
  s.bat_warn = g_warn.active;
  s.bat_cut = g_cut.active;
  s.bat_ov = g_ov.active;
  s.over_temp = g_ntc_hot.active;
  s.cal_ok = g_s.cal_ok;

  const uint32_t dur = (uint32_t)(esp_timer_get_time() - t0);
  s.max_update_us = dur > g_s.max_update_us ? dur : g_s.max_update_us;

  portENTER_CRITICAL(&g_mux);
  g_s = s;
  portEXIT_CRITICAL(&g_mux);
}

void sensing_get(SensSnapshot& out) {
  portENTER_CRITICAL(&g_mux);
  out = g_s;
  portEXIT_CRITICAL(&g_mux);
}

void sensing_clear_trip(uint8_t ch) {
  if (ch > 1) return;
  g_lim[ch].reset();
  g_i_filt[ch] = 0.0f;
  portENTER_CRITICAL(&g_mux);
  g_s.trip[ch] = false;
  g_s.cap_permille[ch] = cfg::CMD_MAX_PERMILLE;
  portEXIT_CRITICAL(&g_mux);
}
