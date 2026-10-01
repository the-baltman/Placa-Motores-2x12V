// PC unit tests for the pure-logic modules. Build/run: see firmware/README.md (run_tests.sh).
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "PlacaProtocol.h"
#include "config.h"
#include "indicator_patterns.h"
#include "motor_math.h"
#include "sensing_math.h"
#include "supervisor.h"

static int g_fail = 0, g_run = 0;
#define CHECK(c) do { ++g_run; if (!(c)) { ++g_fail; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); } } while (0)
#define NEAR(a, b, tol) CHECK(fabsf((float)(a) - (float)(b)) <= (tol))

using namespace placa;

static void test_crc_and_codec() {
  const uint8_t v[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  CHECK(crc16_ccitt(v, 9) == 0x29B1);  // CRC-16/CCITT-FALSE check value

  CmdPacket p = {0x1234, 0xFFFE, FLAG_CLEAR_FAULT, -1000, 1000}, q = {};
  uint8_t buf[CMD_PACKET_LEN];
  CHECK(cmd_encode(p, buf, sizeof buf) == CMD_PACKET_LEN);
  CHECK(buf[0] == 0xB7 && buf[1] == 1 && buf[2] == 0x34 && buf[3] == 0x12);
  CHECK(cmd_decode(buf, sizeof buf, q) == ParseResult::OK);
  CHECK(q.session == p.session && q.seq == p.seq && q.flags == p.flags && q.m1 == -1000 && q.m2 == 1000);

  CHECK(cmd_encode(p, buf, CMD_PACKET_LEN - 1) == 0);
  CmdPacket bad = p; bad.m1 = 1001;
  CHECK(cmd_encode(bad, buf, sizeof buf) == 0);
  bad = p; bad.flags = 0x80;
  CHECK(cmd_encode(bad, buf, sizeof buf) == 0);

  cmd_encode(p, buf, sizeof buf);
  CHECK(cmd_decode(buf, CMD_PACKET_LEN - 1, q) == ParseResult::BAD_LENGTH);
  CHECK(cmd_decode(buf, CMD_PACKET_LEN + 1, q) == ParseResult::BAD_LENGTH);
  CHECK(cmd_decode(nullptr, CMD_PACKET_LEN, q) == ParseResult::BAD_LENGTH);

  // every single-bit flip must be rejected (CRC-16 detects all 1-bit errors)
  for (size_t i = 0; i < CMD_PACKET_LEN; ++i) {
    for (int b = 0; b < 8; ++b) {
      uint8_t c[CMD_PACKET_LEN];
      memcpy(c, buf, sizeof c);
      c[i] ^= (uint8_t)(1u << b);
      CHECK(cmd_decode(c, sizeof c, q) != ParseResult::OK);
    }
  }
  // right CRC but out-of-range command / reserved flags: still rejected
  uint8_t c[CMD_PACKET_LEN];
  memcpy(c, buf, sizeof c);
  c[7] = 0xE9; c[8] = 0x03;  // m1 = 1001
  uint16_t crc = crc16_ccitt(c, 11); c[11] = crc & 0xFF; c[12] = crc >> 8;
  CHECK(cmd_decode(c, sizeof c, q) == ParseResult::BAD_RANGE);
  memcpy(c, buf, sizeof c);
  c[6] = 0x02;
  crc = crc16_ccitt(c, 11); c[11] = crc & 0xFF; c[12] = crc >> 8;
  CHECK(cmd_decode(c, sizeof c, q) == ParseResult::BAD_FLAGS);
  memcpy(c, buf, sizeof c);
  c[0] = 0x00;
  CHECK(cmd_decode(c, sizeof c, q) == ParseResult::BAD_MAGIC);
}

static void test_age() {
  CHECK(age_ms_clamped(1000, 400) == 600);
  CHECK(age_ms_clamped(1000, 1000) == 0);
  CHECK(age_ms_clamped(1000, 1003) == 0);               // F1: packet stored after `now` was captured
  CHECK(age_ms_clamped(5, 0xFFFFFFF0u) == 21);          // millis() wrap
  CHECK(age_ms_clamped(100000, 0) == 100000);           // ordinary old packet is not clamped
}

static void test_seq_filter() {
  SeqFilter f;
  CHECK(seq_accept(f, 7, 100, false));       // first packet
  CHECK(!seq_accept(f, 7, 100, true));       // duplicate
  CHECK(!seq_accept(f, 7, 99, true));        // older
  CHECK(seq_accept(f, 7, 101, true));
  CHECK(!seq_accept(f, 8, 102, true));       // other session while link is up
  CHECK(seq_accept(f, 7, 160, true));        // gap (lost packets) is fine
  CHECK(!seq_accept(f, 7, 160 + 257, true)); // implausible jump
  CHECK(seq_accept(f, 8, 0, false));         // link was down: re-sync to the new session
  CHECK(seq_accept(f, 8, 1, true));
  // wraparound
  SeqFilter w;
  CHECK(seq_accept(w, 1, 65534, false));
  CHECK(seq_accept(w, 1, 65535, true));
  CHECK(seq_accept(w, 1, 0, true));
  CHECK(seq_accept(w, 1, 1, true));
  CHECK(!seq_accept(w, 1, 65535, true));     // replay across the wrap
}

static void test_ramp_and_duty() {
  using namespace motor_math;
  // 1500 permille/s at 10 ms = 15 per step
  CHECK(ramp_step(0, 1000, 1500, 3000, 10) == 15);
  CHECK(ramp_step(990, 1000, 1500, 3000, 10) == 1000);  // no overshoot
  CHECK(ramp_step(100, 0, 1500, 3000, 10) == 70);       // down uses the down rate
  CHECK(ramp_step(100, 0, 1500, 5000, 10) == 50);
  CHECK(ramp_step(10, -500, 1500, 3000, 10) == 0);      // reversal stops exactly at 0
  CHECK(ramp_step(0, -500, 1500, 3000, 10) == -15);
  CHECK(ramp_step(-500, -1000, 1500, 3000, 10) == -515);
  CHECK(ramp_step(5, 5, 1500, 3000, 10) == 5);
  CHECK(ramp_step(0, 100, 1, 1, 10) == 1);              // min step 1
  // full ramp 0 -> 1000 takes ceil(1000/15) = 67 steps = 670 ms
  int16_t v = 0; int n = 0;
  while (v != 1000 && n < 1000) { v = ramp_step(v, 1000, 1500, 3000, 10); ++n; }
  CHECK(n == 67);
  // monotonic, bounded
  v = 1000;
  for (int i = 0; i < 100; ++i) { int16_t nx = ramp_step(v, -1000, 1500, 3000, 10); CHECK(nx <= v && nx >= -1000); v = nx; }

  uint32_t a, b;
  cmd_to_duty(0, a, b);                CHECK(a == 0 && b == 0);
  cmd_to_duty(cfg::CMD_DEADBAND_PERMILLE - 1, a, b); CHECK(a == 0 && b == 0);
  cmd_to_duty(cfg::CMD_DEADBAND_PERMILLE, a, b);     // first live command = 10 % duty
  NEAR(a, 0.10f * cfg::PWM_DUTY_MAX, 1.0f); CHECK(b == 0);
  cmd_to_duty(1000, a, b);             CHECK(a == cfg::PWM_DUTY_MAX && b == 0);
  cmd_to_duty(-1000, a, b);            CHECK(a == 0 && b == cfg::PWM_DUTY_MAX);
  cmd_to_duty(30000, a, b);            CHECK(a == cfg::PWM_DUTY_MAX && b == 0);  // out-of-range clamps
  for (int c = -1000; c <= 1000; ++c) {  // never both legs, never below the minimum when live
    cmd_to_duty((int16_t)c, a, b);
    CHECK(!(a && b));
    if (a || b) CHECK((a > b ? a : b) >= (uint32_t)(0.099f * cfg::PWM_DUTY_MAX));
  }
  NEAR(duty_fraction(1023, 0), 1.0f, 1e-6);
}

static void test_sensing() {
  using namespace sens_math;
  NEAR(vbat_from_adc_mv(1800.0f), 12.0f, 0.01f);  // 12 V * 0.150 = 1.8 V

  // 10 A at D = 0.5 -> V = 10*0.0588*0.5 = 294 mV (+20 mV offset)
  CsEstimate e = cs_estimate(294.0f + 20.0f, 20.0f, 0.5f);
  CHECK(e.valid && !e.is_fault);
  NEAR(e.amps, 10.0f, 0.01f);
  e = cs_estimate(30.0f, 20.0f, 0.05f);           // duty too low
  CHECK(!e.valid);
  e = cs_estimate(10.0f, 20.0f, 0.5f);            // below offset: clamps to 0 A
  CHECK(e.valid); NEAR(e.amps, 0.0f, 1e-6);
  e = cs_estimate(2200.0f, 20.0f, 1.0f);          // driver IS fault level
  CHECK(e.is_fault && !e.valid);
  e = cs_estimate(1900.0f, 0.0f, 1.0f);           // 4 mA IIS(lim) minimum = 2.0 V: 1.9 V must flag a fault (F6)
  CHECK(e.is_fault && !e.valid);
  e = cs_estimate(1700.0f, 0.0f, 1.0f);           // 28.9 A at D=1: below fault level, valid
  CHECK(e.valid); NEAR(e.amps, 28.91f, 0.1f);
  e = cs_estimate(1790.0f, 0.0f, 0.10f);          // large estimate clamps to 41 A
  CHECK(e.valid && e.amps == cfg::CS_I_CLAMP_A);

  HystLow h;
  h.update(10.0f, 9.6f, 9.9f, 20, 500); CHECK(!h.active);
  for (int i = 0; i < 24; ++i) h.update(9.0f, 9.6f, 9.9f, 20, 500);
  CHECK(!h.active);                                 // 480 ms < 500 ms debounce
  h.update(9.0f, 9.6f, 9.9f, 20, 500); CHECK(h.active);
  h.update(9.8f, 9.6f, 9.9f, 20, 500); CHECK(h.active);   // inside hysteresis band
  h.update(9.95f, 9.6f, 9.9f, 20, 500); CHECK(!h.active);
  h.update(9.0f, 9.6f, 9.9f, 20, 500); h.update(10.0f, 9.6f, 9.9f, 20, 500);
  for (int i = 0; i < 24; ++i) h.update(9.0f, 9.6f, 9.9f, 20, 500);
  CHECK(!h.active);                                 // a good sample resets the debounce timer
  HystHigh o;
  o.update(15.6f, 15.5f, 15.0f, 20, 0); CHECK(o.active);
  o.update(15.2f, 15.5f, 15.0f, 20, 0); CHECK(o.active);
  o.update(14.9f, 15.5f, 15.0f, 20, 0); CHECK(!o.active);

  // NTC thresholds: 85 C hot NTC (B3950, 10k@25C) with the other at 25 C -> ~0.295 V (gate C15)
  const float r85 = 10000.0f * expf(3950.0f * (1.0f / 358.15f - 1.0f / 298.15f));
  const float rp = (r85 * 10000.0f) / (r85 + 10000.0f);
  NEAR(3300.0f * rp / (10000.0f + rp), cfg::NTC_CUT_MV, 3.0f);

  CurrentLimiter L;
  for (int i = 0; i < 99; ++i) L.step(true, 13.0f, 10);   // 990 ms over soft limit
  CHECK(L.cap_permille == 1000 && !L.tripped);
  L.step(true, 13.0f, 10); L.step(true, 13.0f, 10);        // crosses 1 s: derate starts
  CHECK(L.cap_permille < 1000);
  for (int i = 0; i < 100; ++i) L.step(true, 13.0f, 10);
  CHECK(L.cap_permille == cfg::DERATE_CAP_MIN_PERMILLE);   // floor
  L.step(false, 99.0f, 10);                                // invalid estimate: ignored
  CHECK(!L.tripped);
  for (int i = 0; i < 400; ++i) L.step(true, 5.0f, 10);
  CHECK(L.cap_permille == 1000);                           // recovers
  L.step(true, 16.0f, 10); L.step(true, 16.0f, 10); CHECK(!L.tripped);
  L.step(true, 16.0f, 10); CHECK(L.tripped);               // 30 ms
  CurrentLimiter L2;
  L2.step(true, 16.0f, 10); L2.step(true, 5.0f, 10); L2.step(true, 16.0f, 10); L2.step(true, 16.0f, 10);
  CHECK(!L2.tripped);                                       // a dip resets the hard timer
  L.reset(); CHECK(!L.tripped && L.cap_permille == 1000);
}

static SupInputs good_inputs() {
  SupInputs in = {};
  in.dt_ms = 10; in.elapsed_ms = 10; in.boot_ok = true; in.estop_ok = true; in.sens_ok = true; in.cal_ok = true;
  in.motors_at_zero = true; in.cap[0] = in.cap[1] = 1000;
  return in;
}

static void test_supervisor() {
  Supervisor s; SupOutputs o;
  SupInputs in = good_inputs();
  for (int i = 0; i < 3; ++i) s.step(in, o);        // warm-up: arming needs ARM_MIN_KICKS healthy kicks

  s.step(in, o);                                    // no link yet
  CHECK(!o.armed && !o.enable_mcu && o.state == SysState::SIN_ENLACE);
  CHECK(o.target[0] == 0 && o.target[1] == 0);

  in.link_alive = true; in.link_ever = true; in.m1 = 800; in.m2 = 800;   // first packet, stick not centred
  s.step(in, o);
  CHECK(!o.armed && !o.enable_mcu && o.target[0] == 0 && o.state == SysState::FAILSAFE_ENLACE);

  in.m1 = in.m2 = 0;                                // neutral: arm
  s.step(in, o);
  CHECK(o.armed && o.armed_edge && o.enable_mcu && o.state == SysState::ENLACE_OK);
  s.step(in, o);
  CHECK(o.armed && !o.armed_edge);

  in.m1 = 500; in.m2 = -300; in.cap[1] = 200;
  s.step(in, o);
  CHECK(o.target[0] == 500 && o.target[1] == -200);  // derate cap applied
  in.cap[1] = 1000;

  in.link_alive = false; in.m1 = in.m2 = 0; in.motors_at_zero = false;   // link lost mid-run
  s.step(in, o);
  CHECK(!o.armed && o.enable_mcu && o.fast_ramp[0] && o.fast_ramp[1] && o.target[0] == 0);  // ramping down, driver still on
  CHECK(o.state == SysState::FAILSAFE_ENLACE);
  in.motors_at_zero = true;
  s.step(in, o);
  CHECK(!o.enable_mcu);                              // EN_MCU drops once at zero

  in.link_alive = true; in.m1 = 1000; in.m2 = 1000;  // link back, stick at full: must NOT move
  s.step(in, o);
  CHECK(!o.armed && o.target[0] == 0 && !o.enable_mcu);
  in.m1 = in.m2 = 0;
  s.step(in, o);
  CHECK(o.armed);

  // E-stop: immediate stop, state ESTOP, needs neutral again
  in.m1 = 600; in.estop_ok = false;
  s.step(in, o);
  CHECK(o.stop_now && !o.enable_mcu && !o.armed && o.state == SysState::ESTOP);
  in.estop_ok = true;
  s.step(in, o);
  CHECK(!o.armed && o.target[0] == 0);
  in.m1 = 0; s.step(in, o); CHECK(o.armed);

  // channel fault: only that channel stops, latched until CLEAR_FAULT at neutral
  in.m1 = 400; in.m2 = 400; in.trip[0] = true;
  s.step(in, o);
  CHECK(o.armed && o.target[0] == 0 && o.target[1] == 400 && o.fault_mask == 1 && o.state == SysState::FALLA_CANAL);
  in.trip[0] = false;
  s.step(in, o);
  CHECK(o.target[0] == 0 && o.fault_mask == 1);      // latched
  in.m1 = in.m2 = 0; in.flags = FLAG_CLEAR_FAULT;
  s.step(in, o);
  CHECK(o.fault_mask == 0 && o.clear_trip[0]);
  in.flags = 0;

  // FLT_ANY low: culprit by CS, or both if unknown
  in.flt_low = true; in.cs_high[1] = true;
  s.step(in, o); CHECK(o.fault_mask == 2);
  in.flt_low = false; in.cs_high[1] = false; in.flags = FLAG_CLEAR_FAULT; s.step(in, o); in.flags = 0; s.step(in, o);
  CHECK(o.fault_mask == 0);
  in.flt_low = true;
  s.step(in, o); CHECK(o.fault_mask == 3);
  in.flt_low = false; in.flags = FLAG_CLEAR_FAULT; s.step(in, o); in.flags = 0;
  CHECK(o.fault_mask == 0);
  in.cs_is_fault[0] = true; s.step(in, o); CHECK(o.fault_mask == 1);
  in.cs_is_fault[0] = false; in.flags = FLAG_CLEAR_FAULT; s.step(in, o); in.flags = 0;

  // CLEAR_FAULT held does not re-clear a fault that appears later (edge only)
  in.flags = FLAG_CLEAR_FAULT; s.step(in, o);
  in.trip[1] = true; s.step(in, o);
  CHECK(o.fault_mask == 2);
  in.trip[1] = false; in.flags = 0; s.step(in, o); in.flags = FLAG_CLEAR_FAULT; s.step(in, o); in.flags = 0;
  CHECK(o.fault_mask == 0);

  // over-temperature, battery cut: gentle stop, no re-arm until clear and neutral
  in.m1 = 300; in.m2 = 300; s.step(in, o); CHECK(!o.armed || o.target[0] == 300);
  in.m1 = in.m2 = 0; s.step(in, o); CHECK(o.armed);
  in.m1 = 300; in.over_temp = true; in.motors_at_zero = false;
  s.step(in, o);
  CHECK(!o.armed && !o.stop_now && o.fast_ramp[0] && o.state == SysState::SOBRETEMP);
  in.over_temp = false; in.bat_cut = true; in.m1 = 0; in.motors_at_zero = true;
  s.step(in, o); CHECK(!o.armed && !o.enable_mcu && o.state == SysState::BATERIA_BAJA);
  in.bat_cut = false; in.bat_warn = true;
  for (int i = 0; i < 4; ++i) s.step(in, o);          // kick run restarts after the over-temp cycle
  CHECK(o.armed && o.state == SysState::BATERIA_BAJA);  // warning alone does not stop the motors
  in.bat_warn = false;

  // overvoltage (F3): EN_MCU cut (coast), disarm, no resume with the stick held, re-arm needs neutral
  in.m1 = 700; in.bat_ov = true; s.step(in, o);
  CHECK(o.stop_now && !o.armed && !o.enable_mcu);
  in.bat_ov = false; s.step(in, o);
  CHECK(!o.armed && o.target[0] == 0 && !o.enable_mcu);
  in.m1 = 0; s.step(in, o); CHECK(o.armed);

  // sensing failure and calibration failure: immediate stop, kick withheld
  in.sens_ok = false; s.step(in, o);
  CHECK(o.stop_now && !o.armed && !o.kick_ok && o.state == SysState::FALLA_SISTEMA);
  in.sens_ok = true; in.cal_ok = false; in.m1 = 0; s.step(in, o);
  CHECK(!o.armed && o.stop_now);
  in.cal_ok = true; in.hw_fault = true; s.step(in, o); CHECK(o.stop_now && !o.kick_ok);
  in.hw_fault = false;

  // init failure: never arms
  Supervisor s2; SupInputs b = good_inputs(); b.boot_ok = false; b.link_alive = true; b.link_ever = true;
  s2.step(b, o);
  CHECK(!o.armed && o.stop_now && !o.kick_ok && o.state == SysState::FALLA_SISTEMA);  // F4

  // F2: arming needs ARM_MIN_KICKS consecutive healthy kicks; a late cycle restarts the count
  Supervisor s4; SupInputs a = good_inputs(); a.link_alive = true; a.link_ever = true;
  for (int i = 0; i < cfg::ARM_MIN_KICKS - 1; ++i) { s4.step(a, o); CHECK(!o.armed); }
  s4.step(a, o); CHECK(o.armed);
  a.late = true; s4.step(a, o); CHECK(o.armed && !o.kick_ok);      // one late cycle: no kick, gap still < 50 ms -> armed
  // F2b: late cycles of 16 ms never exceed 50 ms alone, but the gap between rising kick edges does
  a.elapsed_ms = 16;
  for (int i = 0; i < 4; ++i) s4.step(a, o);                        // 4 x 16 ms with no edge = 64 ms
  CHECK(!o.armed && o.stop_now && !o.enable_mcu);                   // gap > 50 ms with no rising edge
  a.late = false; a.elapsed_ms = 10;
  for (int i = 0; i < cfg::ARM_MIN_KICKS + 2; ++i) s4.step(a, o);   // kicks resume: edges return, neutral present
  CHECK(o.armed);
  a.m1 = 500; a.m2 = 0; a.late = true; a.elapsed_ms = 16;           // stick deflected during a gap: must not resume live
  for (int i = 0; i < 4; ++i) s4.step(a, o);
  CHECK(!o.armed);
  a.late = false; a.elapsed_ms = 10;
  for (int i = 0; i < 8; ++i) s4.step(a, o);
  CHECK(!o.armed && o.target[0] == 0);                              // needs neutral again
  a.m1 = 0; s4.step(a, o); CHECK(o.armed);
  a.late = false; a.stall = true; s4.step(a, o);                   // stall > 50 ms: immediate stop, disarm
  CHECK(!o.armed && o.stop_now && !o.enable_mcu);
  a.stall = false;
  for (int i = 0; i < cfg::ARM_MIN_KICKS - 1; ++i) { s4.step(a, o); CHECK(!o.armed); }
  s4.step(a, o); CHECK(o.armed);
  a.over_temp = true; s4.step(a, o); a.over_temp = false;          // resume after over-temp: kicks restart too
  for (int i = 0; i < cfg::ARM_MIN_KICKS - 1; ++i) { s4.step(a, o); CHECK(!o.armed); }
  s4.step(a, o); CHECK(o.armed);

  // kick stays on when only the link is lost (failsafe is handled, WDT must not trip)
  Supervisor s3; SupInputs k = good_inputs(); s3.step(k, o); CHECK(o.kick_ok);
  // priority: WDT indication below faults, above battery low
  k.wdt_indication = true; k.bat_warn = true; s3.step(k, o); CHECK(o.state == SysState::WDT_DISPARADO);
  k.over_temp = true; s3.step(k, o); CHECK(o.state == SysState::SOBRETEMP);
  k.estop_ok = false; s3.step(k, o); CHECK(o.state == SysState::ESTOP);
}

static void test_patterns() {
  Rgb c = indicator_rgb(SysState::ENLACE_OK, 0, 12345);  CHECK(c.g == 255 && c.r == 0 && c.b == 0);
  c = indicator_rgb(SysState::ESTOP, 0, 999);            CHECK(c.r == 255 && c.g == 0);
  CHECK(indicator_rgb(SysState::INICIANDO, 0, 100).b == 255);
  CHECK(indicator_rgb(SysState::INICIANDO, 0, 600).b == 0);
  // channel 2 fault: 2 flashes in the first 500 ms, dark in the 1 s pause
  CHECK(indicator_rgb(SysState::FALLA_CANAL, 2, 50).r == 255);
  CHECK(indicator_rgb(SysState::FALLA_CANAL, 2, 300).r == 255);
  CHECK(indicator_rgb(SysState::FALLA_CANAL, 2, 200).r == 0);
  CHECK(indicator_rgb(SysState::FALLA_CANAL, 2, 700).r == 0);
  CHECK(indicator_rgb(SysState::FALLA_CANAL, 1, 300).r == 0);  // channel 1 has only one flash
  // battery low: double flash in 2 s
  CHECK(indicator_rgb(SysState::BATERIA_BAJA, 0, 50).b == 255);
  CHECK(indicator_rgb(SysState::BATERIA_BAJA, 0, 250).b == 255);
  CHECK(indicator_rgb(SysState::BATERIA_BAJA, 0, 400).b == 0);
  CHECK(indicator_rgb(SysState::FAILSAFE_ENLACE, 0, 10).r == 255);
  CHECK(indicator_rgb(SysState::FAILSAFE_ENLACE, 0, 130).r == 0);
  CHECK(indicator_rgb(SysState::SIN_ENLACE, 0, 500).r == 255 && indicator_rgb(SysState::SIN_ENLACE, 0, 0).r < 20);
}

int main() {
  test_crc_and_codec();
  test_age();
  test_seq_filter();
  test_ramp_and_duty();
  test_sensing();
  test_supervisor();
  test_patterns();
  printf("%d checks, %d failed\n", g_run, g_fail);
  return g_fail ? 1 : 0;
}
