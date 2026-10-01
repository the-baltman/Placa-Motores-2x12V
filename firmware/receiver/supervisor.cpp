#include "supervisor.h"

#include <PlacaProtocol.h>

#include "config.h"

namespace {

bool is_neutral(int16_t m1, int16_t m2) {
  const int16_t db = cfg::CMD_DEADBAND_PERMILLE;
  return m1 > -db && m1 < db && m2 > -db && m2 < db;
}

int16_t limit_cmd(int16_t cmd, int16_t cap) {
  if (cap < 0) cap = 0;
  if (cmd > cap) return cap;
  if (cmd < -cap) return (int16_t)-cap;
  return cmd;
}

}  // namespace

void Supervisor::reset() { *this = Supervisor(); }

void Supervisor::step(const SupInputs& in, SupOutputs& out) {
  out = SupOutputs();
  out.clear_trip[0] = out.clear_trip[1] = false;
  out.fast_ramp[0] = out.fast_ramp[1] = false;
  out.target[0] = out.target[1] = 0;

  // ---- channel faults (latched)
  for (uint8_t ch = 0; ch < 2; ++ch) {
    if (in.trip[ch] || in.cs_is_fault[ch]) ch_fault_[ch] = true;
  }
  if (in.flt_low) {
    // FLT_ANY does not say which channel: the one whose CS is high is the culprit; if neither
    // can be told apart, latch both (conservative).
    if (!in.cs_high[0] && !in.cs_high[1]) {
      ch_fault_[0] = ch_fault_[1] = true;
    } else {
      if (in.cs_high[0]) ch_fault_[0] = true;
      if (in.cs_high[1]) ch_fault_[1] = true;
    }
  }

  const bool neutral = is_neutral(in.m1, in.m2);
  const bool clear_req = in.link_alive && (in.flags & placa::FLAG_CLEAR_FAULT) != 0;
  if (clear_req && !clear_seen_ && neutral && in.estop_ok && !in.flt_low) {
    for (uint8_t ch = 0; ch < 2; ++ch) {
      if (ch_fault_[ch] && !in.cs_high[ch] && !in.cs_is_fault[ch]) {
        ch_fault_[ch] = false;
        out.clear_trip[ch] = true;
      }
    }
  }
  clear_seen_ = clear_req;

  // ---- kick health. The caller kicks iff kick_ok, so the WDT_KICK level can be mirrored here.
  const bool kick_ok = in.boot_ok && in.sens_ok && !in.over_temp && !in.hw_fault && !in.late && !in.stall;
  // F2b: several late cycles can add up past the hardware monostable's tW with no single cycle over
  // CONTROL_STALL_MS, so measure the gap between effective rising edges on WDT_KICK too.
  const uint32_t gap_ms = since_edge_ms_ + in.elapsed_ms;
  const bool stalled = in.stall || gap_ms > cfg::CONTROL_STALL_MS;
  since_edge_ms_ = gap_ms;
  if (kick_ok) {
    kick_level_ = !kick_level_;
    if (kick_level_) since_edge_ms_ = 0;  // rising edge
  }
  if (kick_ok && !stalled) {
    if (kick_run_ < 255) kick_run_++;
  } else {
    kick_run_ = 0;
  }

  // ---- conditions that forbid motion
  // Overvoltage cuts EN_MCU (coast) and disarms: a ramp-down on a synchronous bridge would regenerate into the bus.
  const bool immediate_block = !in.boot_ok || !in.estop_ok || !in.sens_ok || !in.cal_ok || in.hw_fault || stalled ||
                               in.bat_ov;
  const bool gentle_block = in.over_temp || in.bat_cut;
  const bool blocked = immediate_block || gentle_block;

  // ---- arming / disarming (needs ARM_MIN_KICKS consecutive healthy kicks: the hardware monostable must be running)
  const bool was_armed = armed_;
  if (armed_) {
    if (!in.link_alive || blocked) armed_ = false;
  } else if (in.link_alive && !blocked && neutral && kick_run_ >= cfg::ARM_MIN_KICKS) {
    armed_ = true;
  }
  out.armed = armed_;
  out.armed_edge = armed_ && !was_armed;

  // ---- outputs
  out.stop_now = immediate_block;
  const bool fault_any = ch_fault_[0] || ch_fault_[1];
  out.fault_mask = (uint8_t)((ch_fault_[0] ? 1 : 0) | (ch_fault_[1] ? 2 : 0));

  for (uint8_t ch = 0; ch < 2; ++ch) {
    const int16_t cmd = ch == 0 ? in.m1 : in.m2;
    if (armed_ && !ch_fault_[ch]) {
      out.target[ch] = limit_cmd(cmd, in.cap[ch]);
    } else {
      out.target[ch] = 0;
    }
    // Fast slope whenever the channel is being pulled to zero for a safety reason.
    out.fast_ramp[ch] = !armed_ || ch_fault_[ch];
  }

  if (out.stop_now) {
    out.enable_mcu = false;
  } else if (armed_) {
    out.enable_mcu = true;
  } else {
    // Disarmed without an immediate cut: keep the driver enabled only while the ramp finishes.
    out.enable_mcu = enabled_ && !in.motors_at_zero;
  }
  enabled_ = out.enable_mcu;

  // ---- state for the indicators (priority: ESTOP > FALLA_CANAL > FALLA_SISTEMA > SOBRETEMP >
  //      WDT > BATERIA_BAJA > FAILSAFE_ENLACE > ENLACE_OK > SIN_ENLACE > INICIANDO)
  if (!in.estop_ok) {
    out.state = SysState::ESTOP;
  } else if (fault_any) {
    out.state = SysState::FALLA_CANAL;
  } else if (!in.boot_ok || !in.sens_ok || !in.cal_ok || in.hw_fault) {
    out.state = SysState::FALLA_SISTEMA;
  } else if (in.over_temp) {
    out.state = SysState::SOBRETEMP;
  } else if (in.wdt_indication) {
    out.state = SysState::WDT_DISPARADO;
  } else if (in.bat_warn || in.bat_cut) {
    out.state = SysState::BATERIA_BAJA;
  } else if (armed_) {
    out.state = SysState::ENLACE_OK;
  } else if (in.link_alive || in.link_ever) {
    out.state = SysState::FAILSAFE_ENLACE;  // link lost, or link up but waiting for a neutral command
  } else {
    out.state = SysState::SIN_ENLACE;
  }

  // Watchdog kick health (Pines-ESP32 rule 2): sensing ok and temperature ok. The link is not
  // a condition: losing it is handled by failsafe above, and the kick must continue.
  out.kick_ok = kick_ok;
}
