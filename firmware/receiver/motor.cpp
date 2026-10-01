#include "motor.h"

#include <Arduino.h>

#include "config.h"
#include "motor_math.h"

namespace {

struct Channel {
  uint8_t pin_a, pin_b;
  uint8_t ledc_a, ledc_b;
  int16_t target;
  int16_t ramp;
  uint32_t duty_a, duty_b;
};

Channel g_ch[2] = {
    {cfg::PIN_M1_A, cfg::PIN_M1_B, cfg::LEDC_CH_M1_A, cfg::LEDC_CH_M1_B, 0, 0, 0, 0},
    {cfg::PIN_M2_A, cfg::PIN_M2_B, cfg::LEDC_CH_M2_A, cfg::LEDC_CH_M2_B, 0, 0, 0, 0},
};
bool g_init_ok = false;
bool g_enabled = false;

bool write_duties(Channel& c, uint32_t a, uint32_t b) {
  // Drop the leg that turns off first so A and B are never both high.
  bool ok = true;
  if (a == 0) ok &= ledcWriteChannel(c.ledc_a, 0);
  if (b == 0) ok &= ledcWriteChannel(c.ledc_b, 0);
  if (a != 0) ok &= ledcWriteChannel(c.ledc_a, a);
  if (b != 0) ok &= ledcWriteChannel(c.ledc_b, b);
  if (ok) {
    c.duty_a = a;
    c.duty_b = b;
  }
  return ok;
}

}  // namespace

bool motor_init() {
  // EN_MCU first: a reset left it high-Z (hw pull-down), make it a driven LOW before anything else.
  pinMode(cfg::PIN_EN_MCU, OUTPUT);
  digitalWrite(cfg::PIN_EN_MCU, LOW);
  g_enabled = false;
  g_init_ok = false;

  bool ok = true;
  for (Channel& c : g_ch) {
    ok &= ledcAttachChannel(c.pin_a, cfg::PWM_FREQ_HZ, cfg::PWM_RES_BITS, c.ledc_a);
    ok &= ledcAttachChannel(c.pin_b, cfg::PWM_FREQ_HZ, cfg::PWM_RES_BITS, c.ledc_b);
    c.target = 0;
    c.ramp = 0;
    c.duty_a = c.duty_b = 0;
  }
  if (ok) {
    for (Channel& c : g_ch) ok &= write_duties(c, 0, 0);
  }
  g_init_ok = ok;
  return ok;
}

void motor_set_target(uint8_t ch, int16_t cmd_permille) {
  if (ch > 1) return;
  if (cmd_permille > cfg::CMD_MAX_PERMILLE) cmd_permille = cfg::CMD_MAX_PERMILLE;
  if (cmd_permille < -cfg::CMD_MAX_PERMILLE) cmd_permille = -cfg::CMD_MAX_PERMILLE;
  g_ch[ch].target = cmd_permille;
}

bool motor_update(uint32_t dt_ms, const bool fast[2]) {
  if (!g_init_ok) return false;
  bool ok = true;
  for (uint8_t i = 0; i < 2; ++i) {
    Channel& c = g_ch[i];
    const bool f = fast[i];
    const int32_t up = f ? cfg::RAMP_FAILSAFE_PERMILLE_PER_S : cfg::RAMP_UP_PERMILLE_PER_S;
    const int32_t down = f ? cfg::RAMP_FAILSAFE_PERMILLE_PER_S : cfg::RAMP_DOWN_PERMILLE_PER_S;
    c.ramp = motor_math::ramp_step(c.ramp, c.target, up, down, dt_ms);
    uint32_t a, b;
    motor_math::cmd_to_duty(c.ramp, a, b);
    ok &= write_duties(c, a, b);
  }
  return ok;
}

bool motor_enable(bool enable) {
  if (!enable) {
    for (Channel& c : g_ch) write_duties(c, 0, 0);
    digitalWrite(cfg::PIN_EN_MCU, LOW);
    g_enabled = false;
    return true;
  }
  if (!g_init_ok) return false;
  // Rule 1: duties are already 0 (motor_init), so EN_MCU may rise.
  digitalWrite(cfg::PIN_EN_MCU, HIGH);
  g_enabled = true;
  return true;
}

void motor_stop_now() {
  for (Channel& c : g_ch) {
    c.target = 0;
    c.ramp = 0;
    write_duties(c, 0, 0);
  }
  digitalWrite(cfg::PIN_EN_MCU, LOW);
  g_enabled = false;
}

bool motor_is_enabled() { return g_enabled; }
bool motor_at_zero() { return g_ch[0].ramp == 0 && g_ch[1].ramp == 0; }
int16_t motor_ramp_value(uint8_t ch) { return ch > 1 ? 0 : g_ch[ch].ramp; }
float motor_duty_fraction(uint8_t ch) {
  if (ch > 1) return 0.0f;
  return motor_math::duty_fraction(g_ch[ch].duty_a, g_ch[ch].duty_b);
}
