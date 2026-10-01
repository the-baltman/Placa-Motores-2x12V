// motor.h - 2 bidirectional DC channels: LEDC PWM (timer0, 20 kHz, 10 bit), ramp, EN_MCU.
// H-bridge rule (Pines-ESP32 rule 4): forward = PWM on A with B = 0, reverse = PWM on B with A = 0,
// never both. Both 0 with EN_MCU = 1 is brake; EN_MCU = 0 is coast.
#pragma once
#include <stdint.h>

// Configures the 4 LEDC channels with duty 0 and EN_MCU = LOW. Must be the first user of LEDC
// (the core picks the first free timer = timer0). Returns false if any LEDC call fails.
bool motor_init();

// Target command per channel, permille -1000..+1000. Clamped. Takes effect in motor_update().
void motor_set_target(uint8_t ch, int16_t cmd_permille);

// Advances the ramp by dt_ms and writes the LEDC duties. fast[ch] = use the failsafe slope on that channel.
// Returns false if an LEDC write failed (caller must treat it as a hardware fault).
bool motor_update(uint32_t dt_ms, const bool fast[2]);

// EN_MCU pin. Enabling is refused (returns false) before motor_init() succeeded.
// Disabling never waits: it writes duty 0 first, then drops EN_MCU.
bool motor_enable(bool enable);

// Immediate stop: duty 0 on all legs, ramp state cleared, EN_MCU = LOW (coast).
void motor_stop_now();

bool motor_is_enabled();
bool motor_at_zero();                  // both ramp outputs are exactly 0
int16_t motor_ramp_value(uint8_t ch);  // current ramp output, permille
float motor_duty_fraction(uint8_t ch); // applied PWM duty 0..1 (for IS division)
