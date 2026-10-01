// motor_math.h - pure ramp and command->duty mapping (no hardware; tested on the PC).
#pragma once
#include <stdint.h>

#include "config.h"

namespace motor_math {

inline int32_t iabs(int32_t v) { return v < 0 ? -v : v; }

// One ramp step on a signed command (permille). Growing in magnitude uses up_rate; shrinking,
// or reversing, uses down_rate. A reversal always lands on exactly 0 for one cycle before the
// other direction starts, so the two half-bridge inputs are never driven in opposite order
// without a zero in between.
inline int16_t ramp_step(int16_t cur, int16_t target, int32_t up_rate_pm_s, int32_t down_rate_pm_s,
                         uint32_t dt_ms) {
  if (cur == target) return cur;
  const bool reversing = (cur > 0 && target < 0) || (cur < 0 && target > 0);
  const bool growing = !reversing && iabs(target) > iabs(cur);
  const int32_t rate = growing ? up_rate_pm_s : down_rate_pm_s;
  int32_t step = (rate * (int32_t)dt_ms) / 1000;
  if (step < 1) step = 1;  // never stall on a tiny rate*dt product
  const int32_t goal = reversing ? 0 : target;
  int32_t next = cur;
  if (goal > cur) {
    next = cur + step;
    if (next > goal) next = goal;
  } else {
    next = cur - step;
    if (next < goal) next = goal;
  }
  return (int16_t)next;
}

// Signed command (permille) -> PWM counts on the A and B legs. Deadband, then linear map of the
// remaining span onto [MIN_USEFUL_DUTY, 100 %]. Only one leg is ever non-zero.
inline void cmd_to_duty(int16_t cmd, uint32_t& duty_a, uint32_t& duty_b) {
  duty_a = 0;
  duty_b = 0;
  const int32_t mag = iabs(cmd);
  if (mag < cfg::CMD_DEADBAND_PERMILLE) return;
  int32_t m = mag > 1000 ? 1000 : mag;
  const int32_t span_in = 1000 - cfg::CMD_DEADBAND_PERMILLE;
  const int32_t span_out = 1000 - cfg::MIN_USEFUL_DUTY_PERMILLE;
  const int32_t duty_pm = cfg::MIN_USEFUL_DUTY_PERMILLE + ((m - cfg::CMD_DEADBAND_PERMILLE) * span_out) / span_in;
  const uint32_t counts = (uint32_t)((duty_pm * (int32_t)cfg::PWM_DUTY_MAX + 500) / 1000);
  if (cmd > 0) duty_a = counts; else duty_b = counts;
}

// Applied duty as a fraction 0..1 of the leg that is active (used to divide the IS reading).
inline float duty_fraction(uint32_t duty_a, uint32_t duty_b) {
  const uint32_t d = duty_a > duty_b ? duty_a : duty_b;
  return (float)d / (float)cfg::PWM_DUTY_MAX;
}

}  // namespace motor_math
