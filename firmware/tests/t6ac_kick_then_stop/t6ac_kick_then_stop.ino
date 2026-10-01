// t6ac_kick_then_stop.ino - bench test T6(a)+(c): hardware watchdog after a period of VALID kicks.
//
// *****************************************************************************
// *  SOLO BANCO. SIN MOTORES. NO CARGAR EN EL ROBOT.                          *
// *  This sketch sets EN_MCU = HIGH. Only the hardware watchdog and E-stop    *
// *  stand between EN_MCU and the drivers. Run it with NO motors connected,   *
// *  12 V supply limited to 1 A.                                              *
// *****************************************************************************
//
// Why it exists: a build that hangs from the first cycle never arms, so "EN_DRV = 0" would pass even
// with no watchdog. Here the watchdog is first PROVEN alive (kicks, WDT_OK = 1, EN_DRV = 1) and only
// then starved, which also gives a real "last edge" to time the monostable against.
//
// Sequence:
//   1. LEDC channels GPIO25/26/14/13 at duty 0 (optionally 50 % on M1_A, see M1A_DUTY_PERCENT,
//      to watch the PWM keep running while EN_DRV drops).
//   2. GPIO33 INPUT, GPIO17 OUTPUT LOW, then GPIO27 (EN_MCU) HIGH. Not done if LEDC failed.
//   3. Toggle GPIO17 every 10 ms for 3 s (same cadence as the receiver firmware). Scope: WDT_OK = 1, EN_DRV = 1.
//   4. Drive GPIO17 LOW and leave it fixed forever. Serial prints the time of the last rising edge and
//      of the final (falling) edge, in microseconds since this sketch's setup() started.
//
// Pass criterion (scope on WDT_OK and EN_DRV, trigger on the last rising edge of GPIO17):
//   WDT_OK falls between 69 and 182 ms after the last rising edge, and EN_DRV stays 0 for at least 5 s.
//   Repeat 3 times, power-cycling the board each time.
//
// Pins duplicated from firmware/receiver/config.h (Pines-ESP32 v1.1, frozen); keep in sync by hand.
// Build:  arduino-cli compile --fqbn esp32:esp32:esp32 firmware/tests/t6ac_kick_then_stop
#include <Arduino.h>
#include <esp_timer.h>

static const uint8_t PIN_M1_A = 25;        // LEDC ch0
static const uint8_t PIN_M1_B = 26;        // LEDC ch1
static const uint8_t PIN_M2_A = 14;        // LEDC ch2
static const uint8_t PIN_M2_B = 13;        // LEDC ch3
static const uint8_t PIN_EN_MCU = 27;
static const uint8_t PIN_WDT_KICK = 17;
static const uint8_t PIN_ESTOP_SENSE = 33;
static const uint32_t PWM_FREQ_HZ = 20000;
static const uint8_t PWM_RES_BITS = 10;
static const uint32_t PWM_DUTY_MAX = (1u << PWM_RES_BITS) - 1;
static const uint32_t M1A_DUTY_PERCENT = 0;     // 0 or 50: PWM on M1_A during the test (no motor connected)
static const uint32_t KICK_PERIOD_MS = 10;      // toggle period, same as the receiver control task
static const uint32_t KICK_DURATION_MS = 3000;

static bool g_setup_ok = false;
static int64_t g_t0_us = 0;
static int64_t g_last_rise_us = -1, g_final_us = -1;

void setup() {
  Serial.begin(115200);
  g_t0_us = esp_timer_get_time();

  const uint8_t pins[4] = {PIN_M1_A, PIN_M1_B, PIN_M2_A, PIN_M2_B};
  bool ok = true;
  for (uint8_t ch = 0; ch < 4; ++ch) {
    ok = ok && ledcAttachChannel(pins[ch], PWM_FREQ_HZ, PWM_RES_BITS, ch);
    ok = ok && ledcWriteChannel(ch, 0);
  }
  if (ok && M1A_DUTY_PERCENT > 0) ok = ledcWriteChannel(0, (PWM_DUTY_MAX * M1A_DUTY_PERCENT) / 100);

  pinMode(PIN_ESTOP_SENSE, INPUT);
  pinMode(PIN_WDT_KICK, OUTPUT);
  digitalWrite(PIN_WDT_KICK, LOW);

  g_setup_ok = ok;
  if (!ok) {
    Serial.println("T6ac: LEDC init FAILED, EN_MCU left low. Test invalid.");
    return;
  }
  pinMode(PIN_EN_MCU, OUTPUT);
  digitalWrite(PIN_EN_MCU, HIGH);
  Serial.println("T6ac: EN_MCU=HIGH, kicking GPIO17 every 10 ms for 3 s, then LOW fixed.");
  Serial.flush();  // nothing may be printed during the kick phase

  bool level = false;
  TickType_t last_wake = xTaskGetTickCount();
  const uint32_t toggles = KICK_DURATION_MS / KICK_PERIOD_MS;
  for (uint32_t i = 0; i < toggles; ++i) {
    xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(KICK_PERIOD_MS));
    level = !level;
    digitalWrite(PIN_WDT_KICK, level ? HIGH : LOW);
    if (level) g_last_rise_us = esp_timer_get_time() - g_t0_us;
  }
  digitalWrite(PIN_WDT_KICK, LOW);  // fixed from here on; never touched again
  g_final_us = esp_timer_get_time() - g_t0_us;
}

void loop() {
  static uint32_t n = 0;
  if (g_setup_ok) {
    Serial.printf("t=%us last_rising_edge=%lld us  final_LOW=%lld us  (since setup) EN_MCU=%d WDT_KICK=%d\n", (unsigned)(++n),
                  (long long)g_last_rise_us, (long long)g_final_us, digitalRead(PIN_EN_MCU), digitalRead(PIN_WDT_KICK));
  } else {
    Serial.println("T6ac: test invalid (setup failed)");
  }
  delay(1000);
}
