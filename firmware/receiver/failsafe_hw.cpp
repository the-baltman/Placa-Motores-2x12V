#include "failsafe_hw.h"

#include <Arduino.h>
#include <esp_system.h>

#include "config.h"

namespace {

ResetCause g_cause = ResetCause::OTHER;
bool g_kick_level = false;
bool g_estop_ok = false;  // fail safe: not OK until proven
uint32_t g_estop_ok_ms = 0;
uint32_t g_flt_low_ms = 0;
volatile uint32_t g_flt_edges = 0;  // written only by the ISR

void IRAM_ATTR flt_isr() { g_flt_edges = g_flt_edges + 1; }

ResetCause map_reason(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return ResetCause::POWER_ON;
    case ESP_RST_SW: return ResetCause::SOFTWARE;
    case ESP_RST_TASK_WDT:
    case ESP_RST_INT_WDT:
    case ESP_RST_WDT: return ResetCause::WATCHDOG;
    case ESP_RST_PANIC: return ResetCause::PANIC;
    case ESP_RST_BROWNOUT: return ResetCause::BROWNOUT;
    default: return ResetCause::OTHER;
  }
}

}  // namespace

void failsafe_hw_init() {
  pinMode(cfg::PIN_WDT_KICK, OUTPUT);
  digitalWrite(cfg::PIN_WDT_KICK, LOW);
  g_kick_level = false;

  pinMode(cfg::PIN_ESTOP_SENSE, INPUT);  // external circuit drives it
  pinMode(cfg::PIN_FLT_ANY, INPUT);      // external 10 k pull-up
  attachInterrupt(digitalPinToInterrupt(cfg::PIN_FLT_ANY), flt_isr, FALLING);

  g_cause = map_reason(esp_reset_reason());
  g_estop_ok = false;
  g_estop_ok_ms = 0;
  g_flt_low_ms = 0;
}

ResetCause failsafe_reset_cause() { return g_cause; }

const char* failsafe_reset_cause_name(ResetCause c) {
  switch (c) {
    case ResetCause::POWER_ON: return "power-on";
    case ResetCause::SOFTWARE: return "software";
    case ResetCause::WATCHDOG: return "watchdog";
    case ResetCause::PANIC: return "panic";
    case ResetCause::BROWNOUT: return "brownout";
    default: return "other";
  }
}

void failsafe_hw_kick(bool healthy) {
  if (!healthy) return;
  g_kick_level = !g_kick_level;
  digitalWrite(cfg::PIN_WDT_KICK, g_kick_level ? HIGH : LOW);
}

void failsafe_hw_sample(uint32_t dt_ms, HwInputs& out) {
  const bool estop_high = digitalRead(cfg::PIN_ESTOP_SENSE) == HIGH;
  if (!estop_high) {
    g_estop_ok = false;
    g_estop_ok_ms = 0;
  } else if (!g_estop_ok) {
    g_estop_ok_ms += dt_ms;
    if (g_estop_ok_ms >= cfg::ESTOP_RELEASE_MS) g_estop_ok = true;
  }

  if (digitalRead(cfg::PIN_FLT_ANY) == LOW) {
    g_flt_low_ms += dt_ms;
  } else {
    g_flt_low_ms = 0;
  }

  out.estop_ok = g_estop_ok;
  out.flt_low = g_flt_low_ms >= cfg::FLT_DEBOUNCE_MS;
  out.flt_edges = g_flt_edges;
}
