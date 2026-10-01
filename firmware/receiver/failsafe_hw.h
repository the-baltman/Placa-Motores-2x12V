// failsafe_hw.h - board-level failsafe interface: WDT_KICK, ESTOP_SENSE, FLT_ANY, reset cause.
// The hardware watchdog cuts EN_DRV if WDT_KICK stops toggling; this module only produces the kicks.
#pragma once
#include <stdint.h>

enum class ResetCause : uint8_t {
  POWER_ON,
  SOFTWARE,   // esp_restart()
  WATCHDOG,   // task WDT, interrupt WDT or RTC WDT
  PANIC,      // crash / abort
  BROWNOUT,
  OTHER,
};

struct HwInputs {
  bool estop_ok;       // debounced: true only after ESTOP_SENSE read 1 for ESTOP_RELEASE_MS; false at the first 0
  bool flt_low;        // FLT_ANY low for at least FLT_DEBOUNCE_MS (active-low open-drain)
  uint32_t flt_edges;  // falling edges seen by the ISR since boot (diagnostics)
};

// WDT_KICK as LOW output, ESTOP_SENSE/FLT_ANY as inputs, FLT_ANY ISR (counter only), reset cause.
void failsafe_hw_init();

ResetCause failsafe_reset_cause();
const char* failsafe_reset_cause_name(ResetCause c);

// Toggles WDT_KICK once per call when healthy; holds the level (no edge) otherwise.
// Call exactly once per control cycle (10 ms) from the control task, never from an ISR or timer.
void failsafe_hw_kick(bool healthy);

// Reads and debounces the digital inputs. Call once per control cycle.
void failsafe_hw_sample(uint32_t dt_ms, HwInputs& out);
