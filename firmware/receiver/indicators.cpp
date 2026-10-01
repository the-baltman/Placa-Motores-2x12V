#include "indicators.h"

#include <Arduino.h>

#include "config.h"
#include "indicator_patterns.h"

namespace {

// ---- LED
SysState g_last_state = SysState::INICIANDO;
uint32_t g_state_since_ms = 0;
Rgb g_last_rgb = {255, 255, 255};  // forces the first write
uint32_t g_last_write_ms = 0;
constexpr uint32_t LED_REFRESH_MS = 500;  // rewrite even if unchanged: recovers from an EMI-corrupted frame

// ---- buzzer: non-blocking sequence of beeps
struct Beep {
  uint8_t remaining = 0;  // beeps still to start
  uint32_t on_ms = 0, off_ms = 0;
  bool on = false;
  uint32_t deadline_ms = 0;
} g_beep;
uint32_t g_batlow_next_ms = 0;

void beep_start(uint32_t now_ms, uint8_t count, uint32_t on_ms, uint32_t off_ms) {
  g_beep.remaining = count;
  g_beep.on_ms = on_ms;
  g_beep.off_ms = off_ms;
  g_beep.on = false;
  g_beep.deadline_ms = now_ms;  // start on the next service call
}

void buzzer_write(bool on) { digitalWrite(cfg::PIN_BUZZER, (on && cfg::BUZZER_ENABLED) ? HIGH : LOW); }

void beep_service(uint32_t now_ms) {
  if (g_beep.on) {
    if ((int32_t)(now_ms - g_beep.deadline_ms) >= 0) {
      g_beep.on = false;
      buzzer_write(false);
      g_beep.deadline_ms = now_ms + g_beep.off_ms;
    }
  } else if (g_beep.remaining > 0 && (int32_t)(now_ms - g_beep.deadline_ms) >= 0) {
    g_beep.remaining--;
    g_beep.on = true;
    buzzer_write(true);
    g_beep.deadline_ms = now_ms + g_beep.on_ms;
  }
}

bool is_fault_state(SysState s) {
  return s == SysState::ESTOP || s == SysState::FALLA_CANAL || s == SysState::FALLA_SISTEMA;
}

}  // namespace

void indicators_init() {
  pinMode(cfg::PIN_BUZZER, OUTPUT);
  digitalWrite(cfg::PIN_BUZZER, LOW);
  rgbLedWrite(cfg::PIN_LED_DATA, 0, 0, 0);
  g_last_rgb = RGB_OFF;
  g_beep = Beep();
}

void indicators_update(uint32_t now_ms, SysState state, uint8_t fault_mask, bool armed_edge) {
  if (state != g_last_state) {
    if (is_fault_state(state) && !is_fault_state(g_last_state)) {
      beep_start(now_ms, 3, cfg::BEEP_FAST_ON_MS, cfg::BEEP_FAST_ON_MS);
    }
    if (state == SysState::BATERIA_BAJA) g_batlow_next_ms = now_ms;  // first beep right away
    g_last_state = state;
    g_state_since_ms = now_ms;
  }
  if (armed_edge) beep_start(now_ms, 1, cfg::BEEP_ARM_MS, 0);
  if (state == SysState::BATERIA_BAJA && g_beep.remaining == 0 && !g_beep.on &&
      (int32_t)(now_ms - g_batlow_next_ms) >= 0) {
    beep_start(now_ms, 1, cfg::BEEP_SLOW_ON_MS, 0);
    g_batlow_next_ms = now_ms + cfg::BEEP_BATLOW_PERIOD_MS;
  }
  beep_service(now_ms);

  Rgb c = indicator_rgb(state, fault_mask, now_ms - g_state_since_ms);
  c.r = (uint8_t)((c.r * cfg::LED_BRIGHTNESS_PERCENT) / 100);
  c.g = (uint8_t)((c.g * cfg::LED_BRIGHTNESS_PERCENT) / 100);
  c.b = (uint8_t)((c.b * cfg::LED_BRIGHTNESS_PERCENT) / 100);
  const bool changed = c.r != g_last_rgb.r || c.g != g_last_rgb.g || c.b != g_last_rgb.b;
  if (changed || (now_ms - g_last_write_ms) >= LED_REFRESH_MS) {
    rgbLedWrite(cfg::PIN_LED_DATA, c.r, c.g, c.b);
    g_last_rgb = c;
    g_last_write_ms = now_ms;
  }
}
