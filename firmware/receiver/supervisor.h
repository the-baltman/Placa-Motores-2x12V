// supervisor.h - the safety state machine. Pure logic (no hardware calls): inputs in, decisions out.
// Tested on the PC (firmware/test/test_supervisor.cpp). The control task in receiver.ino feeds it
// once per 10 ms cycle and applies its outputs.
//
// Arming rule: motors run only while ARMED. ARMED starts when the link is alive, no blocking
// condition is active and a NEUTRAL command (both |cmd| < deadband) has been received. It ends on
// link timeout, E-stop, sensing failure, over-temperature or battery cut. After any of those the
// transmitter must send a neutral command again: a stick held at full scale can never restart a motor.
#pragma once
#include <stdint.h>

enum class SysState : uint8_t {
  INICIANDO,      // shown by loop() only before the first control cycle
  SIN_ENLACE,
  ENLACE_OK,
  FAILSAFE_ENLACE,
  ESTOP,
  FALLA_CANAL,
  FALLA_SISTEMA,  // added: init failed (boot_ok false), sensing implausible, calibration failed, LEDC write failed
  SOBRETEMP,
  BATERIA_BAJA,
  WDT_DISPARADO,
};

struct SupInputs {
  uint32_t dt_ms;
  bool boot_ok;        // motor, sensing and ESP-NOW initialised
  bool link_alive;
  bool link_ever;
  int16_t m1, m2;      // latest command, permille
  uint8_t flags;
  bool estop_ok;
  bool flt_low;        // debounced FLT_ANY (active low)
  bool sens_ok;
  bool cal_ok;
  bool bat_warn, bat_cut, bat_ov;
  bool over_temp;
  bool cs_high[2];     // CS above the channel-identification margin
  bool cs_is_fault[2];
  bool trip[2];        // soft-limit hard trip (> 15 A)
  int16_t cap[2];      // derate cap, permille
  bool motors_at_zero;
  bool hw_fault;       // motor_update() failed to write an LEDC duty
  uint32_t elapsed_ms; // real time since the previous control cycle (not the nominal period)
  bool late;           // this control cycle started late: the WDT kick is withheld
  bool stall;          // control period > CONTROL_STALL_MS (the supervisor also tracks the gap between rising kick edges)
  bool wdt_indication; // reset cause was a watchdog and boot was < 10 s ago
};

struct SupOutputs {
  bool enable_mcu;        // desired EN_MCU
  bool stop_now;          // duty 0 and EN_MCU low immediately
  bool fast_ramp[2];      // use the failsafe slope on this channel
  int16_t target[2];      // command after cap and fault masking, permille
  bool clear_trip[2];     // reset the sensing limiter for this channel
  SysState state;
  uint8_t fault_mask;     // bit0 = channel 1, bit1 = channel 2 latched faults
  bool armed;
  bool armed_edge;        // true for exactly one step when arming
  bool kick_ok;           // supervisor-side health for the hardware watchdog kick
};

class Supervisor {
 public:
  void reset();
  void step(const SupInputs& in, SupOutputs& out);

 private:
  bool armed_ = false;
  bool enabled_ = false;
  bool ch_fault_[2] = {false, false};
  bool clear_seen_ = false;
  uint8_t kick_run_ = 0;     // consecutive kicks performed
  bool kick_level_ = false;  // mirror of the WDT_KICK pin level (toggles on every performed kick)
  uint32_t since_edge_ms_ = 0;  // time since the last rising edge on WDT_KICK  // CLEAR_FAULT is acted on at its rising edge only
};
