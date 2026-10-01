// sensing.h - ADC1 sensing: motor current (IS via CS, divided by duty), battery, NTC, soft limits.
// All inputs are ADC1 (ADC2 is unusable with WiFi/ESP-NOW). Called from the 10 ms control task.
#pragma once
#include <stdint.h>

struct SensSnapshot {
  float vbat_v;          // battery voltage after divider correction, V
  float ntc_mv;          // NTC node, mV
  float cs_mv[2];        // raw ADC mV at CS_M1 / CS_M2 (offset not removed)
  float i_amps[2];       // estimated average motor current, A (0 while i_valid is false)
  bool i_valid[2];       // false at duty < 10 % (IS sample too small) or driver fault
  bool cs_is_fault[2];   // IS above the driver-fault level
  bool cs_high[2];       // CS above 1.0 V: culprit candidate when FLT_ANY is low
  bool sens_ok;          // VBAT and NTC readings are plausible
  bool cal_ok;           // zero-current calibration succeeded
  bool bat_warn, bat_cut, bat_ov;
  bool over_temp;        // NTC cut (latched with hysteresis until cooled to the release level)
  int16_t cap_permille[2];  // soft-limit cap on |command| (1000 = no derate)
  bool trip[2];          // hard limit exceeded: channel off until sensing_clear_trip()
  uint32_t max_update_us;   // worst sensing_update() duration since boot (bench: must be << 10 ms)
};

bool sensing_init();                    // ADC attenuation, first full read. false = implausible readings
bool sensing_calibrate_zero();          // motors off, EN_MCU = 0: measures CS offset. false = out of range
// duty[ch] = applied PWM duty fraction 0..1 (from motor_duty_fraction).
void sensing_update(uint32_t dt_ms, const float duty[2]);
void sensing_get(SensSnapshot& out);
void sensing_clear_trip(uint8_t ch);
