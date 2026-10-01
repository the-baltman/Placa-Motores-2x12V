// receiver.ino - Placa-Motores-2x12V, ESP32-WROOM-32 (Arduino-ESP32 core 3.x).
// Two 12 V DC motors over ESP-NOW. Safety design: see supervisor.h and README.md.
//
// Tasks
//   controlTask (core 1, prio 4, 10 ms, vTaskDelayUntil): inputs -> supervisor -> motors -> WDT kick.
//   loop()      (core 1, prio 1): LED, buzzer, serial status. Indication only; cannot move a motor.
//
// Boot order (Pines-ESP32 rule 1): EN_MCU low and LEDC at duty 0 BEFORE anything can enable a driver.
#include <Arduino.h>
#include <esp_task_wdt.h>

#include "config.h"
#include "espnow_link.h"
#include "failsafe_hw.h"
#include "indicators.h"
#include "motor.h"
#include "sensing.h"
#include "supervisor.h"

static_assert(configTICK_RATE_HZ == 1000, "control period assumes a 1 kHz FreeRTOS tick");

namespace {

struct Status {
  SysState state;
  uint8_t fault_mask;
  uint32_t armed_edges;  // counter, so loop() never misses the one-shot
  bool armed;
  int16_t ramp[2];
  float i_amps[2];
  float vbat_v;
  float ntc_mv;
  uint32_t cycle_max_us;
  uint32_t late_cycles;
  uint32_t sens_max_us;
  uint32_t rx_ok, rx_bad, rx_old, rx_foreign, flt_edges;
  bool cal_ok, sens_ok;
};

Status g_status = {};
portMUX_TYPE g_status_mux = portMUX_INITIALIZER_UNLOCKED;
bool g_boot_ok = false;
Supervisor g_sup;

void configure_unused_pins() {
  // Declared in the pin table but not driven by firmware v1: plain inputs, no pull.
  const uint8_t unused[] = {cfg::PIN_ENC1_A, cfg::PIN_ENC1_B, cfg::PIN_ENC2_A, cfg::PIN_ENC2_B,
                            cfg::PIN_AUX1, cfg::PIN_AUX2};
  for (uint8_t p : unused) pinMode(p, INPUT);
}

void controlTask(void*) {
  bool hw_fault = false;
  // This task must feed the internal task watchdog every cycle; if it cannot subscribe, refuse to run motors.
  if (esp_task_wdt_add(nullptr) != ESP_OK) hw_fault = true;
  TickType_t last_wake = xTaskGetTickCount();
  uint32_t prev_us = (uint32_t)micros();
  uint32_t armed_edges = 0, late_cycles = 0, cycle_max_us = 0;
  const bool wdt_reset = failsafe_reset_cause() == ResetCause::WATCHDOG || failsafe_reset_cause() == ResetCause::PANIC;

  for (;;) {
    const BaseType_t delayed = xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(cfg::CONTROL_PERIOD_MS));
    const uint32_t now_us = (uint32_t)micros();
    const uint32_t now_ms = millis();
    const uint32_t period_us = now_us - prev_us;
    prev_us = now_us;
    const bool late = (delayed == pdFALSE) || period_us > (cfg::CONTROL_PERIOD_MS + cfg::CONTROL_LATE_MS) * 1000u;
    if (late) late_cycles++;
    if (period_us > cycle_max_us) cycle_max_us = period_us;

    HwInputs hw;
    failsafe_hw_sample(cfg::CONTROL_PERIOD_MS, hw);

    float duty[2] = {motor_duty_fraction(0), motor_duty_fraction(1)};
    sensing_update(cfg::CONTROL_PERIOD_MS, duty);
    SensSnapshot s;
    sensing_get(s);

    LinkSnapshot link;
    espnow_link_poll(now_ms, link);

    SupInputs in = {};
    in.dt_ms = cfg::CONTROL_PERIOD_MS;
    in.boot_ok = g_boot_ok;
    in.link_alive = link.alive;
    in.link_ever = link.ever_linked;
    in.m1 = link.m1;
    in.m2 = link.m2;
    in.flags = link.flags;
    in.estop_ok = hw.estop_ok;
    in.flt_low = hw.flt_low;
    in.sens_ok = s.sens_ok;
    in.cal_ok = s.cal_ok;
    in.bat_warn = s.bat_warn;
    in.bat_cut = s.bat_cut;
    in.bat_ov = s.bat_ov;
    in.over_temp = s.over_temp;
    for (uint8_t ch = 0; ch < 2; ++ch) {
      in.cs_high[ch] = s.cs_high[ch];
      in.cs_is_fault[ch] = s.cs_is_fault[ch];
      in.trip[ch] = s.trip[ch];
      in.cap[ch] = s.cap_permille[ch];
    }
    in.motors_at_zero = motor_at_zero();
    in.hw_fault = hw_fault;
    in.late = late;
    in.elapsed_ms = (period_us + 500) / 1000;
    in.stall = period_us > cfg::CONTROL_STALL_MS * 1000u;
    in.wdt_indication = wdt_reset && now_ms < cfg::WDT_BOOT_INDICATION_MS;

    SupOutputs out;
    g_sup.step(in, out);
    for (uint8_t ch = 0; ch < 2; ++ch) {
      if (out.clear_trip[ch]) sensing_clear_trip(ch);
    }

    if (out.stop_now) {
      motor_stop_now();
    } else {
      motor_set_target(0, out.target[0]);
      motor_set_target(1, out.target[1]);
      if (out.enable_mcu && !motor_is_enabled()) {
        if (!motor_enable(true)) hw_fault = true;
      }
      if (!motor_update(cfg::CONTROL_PERIOD_MS, out.fast_ramp)) {
        hw_fault = true;  // sticky until reboot: an LEDC write failure is not recoverable in the field
        motor_stop_now();
      }
      if (!out.enable_mcu && motor_is_enabled()) motor_enable(false);
    }
    if (out.armed_edge) armed_edges++;

    // out.kick_ok already folds in lateness, sensing, temperature and LEDC health (Pines rule 2).
    failsafe_hw_kick(out.kick_ok);
    esp_task_wdt_reset();

    Status st = {};
    st.state = out.state;
    st.fault_mask = out.fault_mask;
    st.armed_edges = armed_edges;
    st.armed = out.armed;
    st.ramp[0] = motor_ramp_value(0);
    st.ramp[1] = motor_ramp_value(1);
    st.i_amps[0] = s.i_amps[0];
    st.i_amps[1] = s.i_amps[1];
    st.vbat_v = s.vbat_v;
    st.ntc_mv = s.ntc_mv;
    st.cycle_max_us = cycle_max_us;
    st.late_cycles = late_cycles;
    st.sens_max_us = s.max_update_us;
    st.rx_ok = link.rx_ok;
    st.rx_bad = link.rx_bad;
    st.rx_old = link.rx_old;
    st.rx_foreign = link.rx_foreign;
    st.flt_edges = hw.flt_edges;
    st.cal_ok = s.cal_ok;
    st.sens_ok = s.sens_ok;
    portENTER_CRITICAL(&g_status_mux);
    g_status = st;
    portEXIT_CRITICAL(&g_status_mux);
  }
}

void handle_serial() {
  // "t <ms>" sets the link timeout at runtime (clamped 100..2000 ms).
  static char buf[16];
  static uint8_t n = 0;
  while (Serial.available() > 0) {
    const char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      buf[n] = 0;
      if (n > 2 && buf[0] == 't' && buf[1] == ' ') {
        espnow_link_set_timeout_ms((uint32_t)atoi(buf + 2));
        Serial.printf("link timeout = %u ms\n", (unsigned)espnow_link_timeout_ms());
      }
      n = 0;
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = ch;
    }
  }
}

}  // namespace

void setup() {
  // 1. Safe outputs first.
  const bool motor_ok = motor_init();
  failsafe_hw_init();
  indicators_init();
  configure_unused_pins();
  Serial.begin(115200);
  Serial.printf("\nPlaca-Motores-2x12V receiver. reset cause: %s\n", failsafe_reset_cause_name(failsafe_reset_cause()));
  if (!motor_ok) Serial.println("ERROR: motor_init (LEDC) failed");

  // 2. Sensing, with drivers off (EN_MCU low): zero-current calibration.
  const bool sens_ok = sensing_init();
  if (!sens_ok) Serial.println("ERROR: VBAT/NTC readings implausible at boot");
  if (!sensing_calibrate_zero()) Serial.println("ERROR: CS zero calibration out of range (offset > limit)");

  // 3. Let the E-stop debounce settle before the control task starts judging it.
  HwInputs hw;
  for (int i = 0; i < 8; ++i) {
    failsafe_hw_sample(10, hw);
    delay(10);
  }

  // 4. Radio.
  bool filter_zero = true;
  for (uint8_t b : cfg::TX_MAC_FILTER) filter_zero = filter_zero && b == 0;
  if (filter_zero) Serial.println("WARN: bench mode: TX_MAC_FILTER is all zeros, ANY transmitter on this channel is accepted");
  const bool link_ok = espnow_link_init();
  if (!link_ok) Serial.println("ERROR: ESP-NOW init failed");
  char mac[18];
  espnow_link_mac_string(mac, sizeof(mac));
  Serial.printf("MAC %s  channel %u  timeout %u ms\n", mac, (unsigned)cfg::ESPNOW_CHANNEL, (unsigned)espnow_link_timeout_ms());

  // 5. Internal task watchdog: 1 s, panic -> reset. Only subscribed tasks are watched.
  esp_task_wdt_config_t wcfg = {};
  wcfg.timeout_ms = cfg::TWDT_TIMEOUT_MS;
  wcfg.idle_core_mask = 0;
  wcfg.trigger_panic = true;
  const bool wdt_ok = esp_task_wdt_reconfigure(&wcfg) == ESP_OK;
  if (!wdt_ok) Serial.println("ERROR: esp_task_wdt_reconfigure failed");

  g_boot_ok = motor_ok && link_ok && wdt_ok && sens_ok;
  g_sup.reset();

  const BaseType_t created = xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr, 4, nullptr, 1);
  if (created != pdPASS) {
    g_boot_ok = false;  // no control task = no kicks: the hardware watchdog keeps the drivers off
    Serial.println("ERROR: control task not created");
  }
}

void loop() {
  static uint32_t last_print_ms = 0;
  static uint32_t seen_armed_edges = 0;
  static Status st = {};
  static bool first = true;
  const uint32_t now = millis();

  Status cur;
  portENTER_CRITICAL(&g_status_mux);
  cur = g_status;
  portEXIT_CRITICAL(&g_status_mux);
  if (first) {  // before the first control cycle g_status is zeroed: show INICIANDO, ignore edges
    first = false;
    st.state = SysState::INICIANDO;
  }
  if (cur.cycle_max_us != 0 || cur.armed_edges != 0) st = cur;

  if (!g_boot_ok) st.state = SysState::FALLA_SISTEMA;  // e.g. control task never started
  const bool edge = st.armed_edges != seen_armed_edges;
  seen_armed_edges = st.armed_edges;
  indicators_update(now, st.state, st.fault_mask, edge);
  handle_serial();

  if (now - last_print_ms >= 1000) {
    last_print_ms = now;
    Serial.printf("st=%u armed=%d ramp=%d/%d I=%.1f/%.1fA vbat=%.2fV ntc=%.0fmV rx ok/bad/old/foreign=%u/%u/%u/%u flt=%u ctl_max=%uus late=%u sens_max=%uus cal=%d sens=%d\n",
                  (unsigned)st.state, st.armed, st.ramp[0], st.ramp[1], st.i_amps[0], st.i_amps[1], st.vbat_v,
                  st.ntc_mv, (unsigned)st.rx_ok, (unsigned)st.rx_bad, (unsigned)st.rx_old, (unsigned)st.rx_foreign,
                  (unsigned)st.flt_edges, (unsigned)st.cycle_max_us, (unsigned)st.late_cycles,
                  (unsigned)st.sens_max_us, st.cal_ok, st.sens_ok);
  }
  delay(10);
}
