// config.h - every pin, limit and gain of the receiver firmware, in one place.
// Pins: Pines-ESP32 v1.1 (frozen, C Morty gate 2026-09-30). Module: ESP32-WROOM-32 ONLY
// (on WROVER, GPIO16/17 are PSRAM). GPIO12 (MTDI strapping) is never touched by this firmware.
// Pure constexprs (no Arduino headers) so the PC tests can include it.
#pragma once
#include <stdint.h>

namespace cfg {

// ------------------------------------------------------------------ pins (GPIO numbers)
constexpr uint8_t PIN_M1_A = 25;         // OUT, LEDC ch0, IBT-2 #1 RPWM (forward)
constexpr uint8_t PIN_M1_B = 26;         // OUT, LEDC ch1, IBT-2 #1 LPWM (reverse)
constexpr uint8_t PIN_M2_A = 14;         // OUT, LEDC ch2, IBT-2 #2 RPWM
constexpr uint8_t PIN_M2_B = 13;         // OUT, LEDC ch3, IBT-2 #2 LPWM
constexpr uint8_t PIN_EN_MCU = 27;       // OUT, request to enable both drivers (AND3 with WDT_OK, ESTOP_OK)
constexpr uint8_t PIN_CS_M1 = 36;        // IN analog ADC1_CH0 (input-only, no internal pulls)
constexpr uint8_t PIN_CS_M2 = 39;        // IN analog ADC1_CH3
constexpr uint8_t PIN_VBAT_SENSE = 34;   // IN analog ADC1_CH6
constexpr uint8_t PIN_NTC_MAX = 35;      // IN analog ADC1_CH7
constexpr uint8_t PIN_FLT_ANY = 4;       // IN, active LOW (open-drain, external pull-up)
constexpr uint8_t PIN_ESTOP_SENSE = 33;  // IN, 1 = loop closed (E-stop OK)
constexpr uint8_t PIN_LED_DATA = 32;     // OUT, WS2812 via 74AHCT1G125
constexpr uint8_t PIN_WDT_KICK = 17;     // OUT, toggled by software from the control task
constexpr uint8_t PIN_BUZZER = 2;        // OUT, strapping: must be LOW at boot (hw pull-down)
// Declared but unused in firmware v1 (configured as plain inputs so nothing drives them):
constexpr uint8_t PIN_ENC1_A = 18;
constexpr uint8_t PIN_ENC1_B = 19;
constexpr uint8_t PIN_ENC2_A = 23;
constexpr uint8_t PIN_ENC2_B = 16;
constexpr uint8_t PIN_AUX1 = 5;   // J_AUX, no function in v1, input without pull
constexpr uint8_t PIN_AUX2 = 15;  // J_AUX, no function in v1, input without pull
// GPIO 0/1/3 (boot/UART0) and 12 (MTDI strapping): not used.

// ------------------------------------------------------------------ control loop timing
constexpr uint32_t CONTROL_PERIOD_MS = 10;  // control task period, timer-driven (vTaskDelayUntil)
constexpr uint32_t CONTROL_LATE_MS = 5;     // a cycle that starts > this late is reported and does not kick the WDT
constexpr uint32_t CONTROL_STALL_MS = 50;   // max control period AND max time between rising edges on WDT_KICK: above this = immediate stop + disarm (hw monostable tW min = 69 ms)
constexpr uint8_t ARM_MIN_KICKS = 4;        // consecutive healthy kicks (>= 1 rising edge on WDT_KICK) before arming (Pines rule 8)
constexpr uint32_t TWDT_TIMEOUT_MS = 1000;  // internal task watchdog: panic + reset if control task hangs
constexpr uint32_t WDT_BOOT_INDICATION_MS = 10000;  // WDT_DISPARADO LED visible this long after boot

// ------------------------------------------------------------------ PWM (LEDC)
constexpr uint32_t PWM_FREQ_HZ = 20000;  // Hz, inaudible; BTS7960 accepts up to 25 kHz
constexpr uint8_t PWM_RES_BITS = 10;     // bits
constexpr uint32_t PWM_DUTY_MAX = (1u << PWM_RES_BITS) - 1;  // 1023 counts
constexpr uint8_t LEDC_CH_M1_A = 0, LEDC_CH_M1_B = 1, LEDC_CH_M2_A = 2, LEDC_CH_M2_B = 3;

// ------------------------------------------------------------------ command shaping (permille = 1/1000 of full scale)
constexpr int16_t CMD_MAX_PERMILLE = 1000;     // full scale of a command
constexpr int16_t CMD_DEADBAND_PERMILLE = 30;   // |cmd| below this = 0 (joystick noise)
constexpr int16_t MIN_USEFUL_DUTY_PERMILLE = 100;  // BTS7960 turn-on delay up to 4.5 us = 9 % of 50 us: below ~10 % is useless
// Ramp slopes, permille of command per second.
constexpr int32_t RAMP_UP_PERMILLE_PER_S = 1500;       // 0 -> 100 % in ~0.67 s
constexpr int32_t RAMP_DOWN_PERMILLE_PER_S = 3000;     // normal decel / direction reversal
constexpr int32_t RAMP_FAILSAFE_PERMILLE_PER_S = 5000; // link loss and channel faults: 100 % -> 0 in 0.2 s

// ------------------------------------------------------------------ ESP-NOW link
constexpr uint32_t LINK_TIMEOUT_MS_DEFAULT = 300;  // no valid packet for this long -> failsafe (runtime-configurable)
constexpr uint32_t LINK_TIMEOUT_MS_MIN = 100;      // setter clamps to this range
constexpr uint32_t LINK_TIMEOUT_MS_MAX = 2000;
constexpr uint8_t ESPNOW_CHANNEL = 1;  // must equal the transmitter's channel
// Only accept packets from this transmitter MAC. All zeros = accept any source (bench only).
constexpr uint8_t TX_MAC_FILTER[6] = {0, 0, 0, 0, 0, 0};

// ------------------------------------------------------------------ current sensing (Calculos.md sec. 5)
constexpr float CS_VOLT_PER_AMP = 0.0588f;  // V/A: R_IS effective 500 ohm, IS/IL = 1/8500
constexpr float CS_MIN_DUTY_VALID = 0.10f;  // below this D the IS sample is too small to divide by
constexpr float CS_ADC_LINEAR_MAX_MV = 2450.0f;  // documentation only: ADC 11 dB linear range top (not used in code)
constexpr float CS_IS_FAULT_MV = 1800.0f;   // BTS7960 IIS(lim) min 4 mA * 500 ohm = 2.0 V (typ 4.5 mA = 2.25 V): flag below the minimum
constexpr float CS_I_CLAMP_A = 41.0f;       // sensing is linear up to ~41 A (Calculos.md)
constexpr float CS_OFFSET_MAX_MV = 100.0f;  // a zero-current offset above this = calibration failed (expected ~20 mV GND offset)
constexpr float CS_HW_COMPARATOR_MV = 1180.0f;  // documentation only (not used in code). LM393 reference: fault if CS > 1.18 V (= D*I > 20 A)
constexpr float CS_FLT_CHANNEL_MARGIN_MV = 1000.0f; // FLT_ANY low: channel with CS above this is the culprit
constexpr uint8_t CS_OVERSAMPLE = 8;        // ADC samples per CS read (plus IIR below)
constexpr float CS_IIR_ALPHA = 0.25f;       // I_est low-pass per 10 ms cycle (tau ~ 30 ms)
constexpr uint16_t CS_CAL_SAMPLES = 64;     // zero-current calibration samples (motors stopped, EN_MCU = 0)
// Soft limits (Indicadores-y-failsafe.md sec. 3)
constexpr float I_SOFT_LIMIT_A = 12.0f;     // above this for I_SOFT_TIME_MS -> derate
constexpr uint32_t I_SOFT_TIME_MS = 1000;
constexpr uint32_t I_HARD_TIME_MS = 30;     // hard-limit estimate must persist this long (3 cycles) to trip
constexpr float I_SOFT_RELEASE_A = 11.0f;   // derate recovery below this
constexpr float I_HARD_LIMIT_A = 15.0f;     // above this (valid estimate) -> channel off, latched
constexpr int16_t DERATE_STEP_PERMILLE = 20;    // cap reduction per control cycle while over soft limit
constexpr int16_t DERATE_RECOVER_PERMILLE = 2;  // cap recovery per control cycle
constexpr int16_t DERATE_CAP_MIN_PERMILLE = 200;  // derate never takes the cap below this (hard limit handles the rest)
constexpr uint32_t FLT_DEBOUNCE_MS = 20;    // FLT_ANY low this long = fault
constexpr uint32_t ESTOP_RELEASE_MS = 50;   // E-stop must read OK this long to count as released

// ------------------------------------------------------------------ battery (Calculos.md sec. 6)
constexpr bool BATTERY_IS_LIPO = true;  // true: LiPo 3S, false: lead-acid 12 V
constexpr float VBAT_DIV_K = 12.0f / (68.0f + 12.0f);  // 68k/12k divider = 0.150
constexpr float VBAT_WARN_V = BATTERY_IS_LIPO ? 9.9f : 11.0f;   // BATERIA_BAJA indication
constexpr float VBAT_CUT_V = BATTERY_IS_LIPO ? 9.6f : 10.5f;    // soft cut (motors off, latch until re-arm)
constexpr float VBAT_HYST_V = 0.3f;
constexpr uint32_t VBAT_CUT_DEBOUNCE_MS = 500;  // ride through load sag
constexpr float VBAT_OV_V = 15.5f;        // regeneration: cut EN_MCU (coast) and disarm, do not push more
constexpr float VBAT_OV_RELEASE_V = 15.0f;
constexpr float VBAT_PLAUSIBLE_MIN_V = 5.0f;   // below/above = sensor fault, fail safe
constexpr float VBAT_PLAUSIBLE_MAX_V = 20.0f;

// ------------------------------------------------------------------ NTC (Calculos.md sec. 7, gate C15)
// 10 k pull-up to 3V3 + two 10 k B3950 NTC in parallel. V rises with cooler temperature.
constexpr float NTC_CUT_MV = 295.0f;      // 85 C on the hot NTC with the other at 25 C (C Morty C15)
constexpr float NTC_RELEASE_MV = 430.0f;  // 70 C (upper bound of the 0.27-0.43 V band)
constexpr float NTC_PLAUSIBLE_MIN_MV = 50.0f;    // below = shorted sensor
constexpr float NTC_PLAUSIBLE_MAX_MV = 3000.0f;  // above = open sensor (bench without NTC: set NTC_FAULT_ENABLED = false)
constexpr bool NTC_FAULT_ENABLED = true;

// ------------------------------------------------------------------ indicators
constexpr uint8_t LED_BRIGHTNESS_PERCENT = 25;  // max WS2812 brightness (eyes and current)
constexpr uint32_t BEEP_ARM_MS = 150;
constexpr uint32_t BEEP_FAST_ON_MS = 100;
constexpr uint32_t BEEP_SLOW_ON_MS = 400;
constexpr uint32_t BEEP_BATLOW_PERIOD_MS = 5000;
constexpr bool BUZZER_ENABLED = true;

}  // namespace cfg
