// indicators.h - WS2812 status LED (GPIO32) and active buzzer (GPIO2, plain on/off, no LEDC).
// Non-blocking; call every control period from a low-priority context (loop()). Indication only:
// nothing here can affect the motors.
#pragma once
#include <stdint.h>

#include "supervisor.h"

// LED off, buzzer driven LOW (GPIO2 is a strapping pin: it must never be held high at boot).
void indicators_init();

// fault_mask as in SupOutputs. armed_edge: one-shot "armed" beep.
void indicators_update(uint32_t now_ms, SysState state, uint8_t fault_mask, bool armed_edge);
