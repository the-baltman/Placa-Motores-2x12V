// t6b_no_kick.ino - bench test T6(b): hardware watchdog with a LIVE ESP32 that never kicks.
//
// *****************************************************************************
// *  SOLO BANCO. SIN MOTORES. NO CARGAR EN EL ROBOT.                          *
// *  This sketch ENABLES the drivers request (EN_MCU = HIGH) while WDT_KICK   *
// *  is held LOW. Only the hardware watchdog stands between EN_MCU and the    *
// *  drivers. Run it with NO motors connected, 12 V supply limited to 1 A.    *
// *****************************************************************************
//
// Sequence (Electronica, ERC-DRC-y-pruebas.md, "Correcciones y metodos de medicion"):
//   1. LEDC channels on GPIO25/26/14/13 with duty 0 (PWM stays running at 0 %).
//   2. GPIO17 (WDT_KICK) OUTPUT LOW, fixed: zero edges, never touched again.
//   3. GPIO33 (ESTOP_SENSE) INPUT.
//   4. GPIO27 (EN_MCU) HIGH.
// loop() only prints one line per second. It never touches GPIO17.
//
// Pass criterion (scope on EN_DRV and WDT_OK): EN_DRV = 0 sustained >= 5 s,
// repeated 3 times, power-cycling the board each time.
//
// Pins duplicated from firmware/receiver/config.h (Pines-ESP32 v1.1, frozen). A sketch folder cannot
// portably include ../../receiver/config.h, so keep these in sync by hand.
// Build:  arduino-cli compile --fqbn esp32:esp32:esp32 firmware/tests/t6b_no_kick
#include <Arduino.h>

static const uint8_t PIN_M1_A = 25;        // LEDC ch0
static const uint8_t PIN_M1_B = 26;        // LEDC ch1
static const uint8_t PIN_M2_A = 14;        // LEDC ch2
static const uint8_t PIN_M2_B = 13;        // LEDC ch3
static const uint8_t PIN_EN_MCU = 27;
static const uint8_t PIN_WDT_KICK = 17;    // held LOW for the whole test
static const uint8_t PIN_ESTOP_SENSE = 33;
static const uint32_t PWM_FREQ_HZ = 20000;
static const uint8_t PWM_RES_BITS = 10;

static bool g_setup_ok = false;

void setup() {
  Serial.begin(115200);

  // 1. PWM channels at duty 0.
  const uint8_t pins[4] = {PIN_M1_A, PIN_M1_B, PIN_M2_A, PIN_M2_B};
  bool ok = true;
  for (uint8_t ch = 0; ch < 4; ++ch) {
    ok = ok && ledcAttachChannel(pins[ch], PWM_FREQ_HZ, PWM_RES_BITS, ch);
    ok = ok && ledcWriteChannel(ch, 0);
  }

  // 2. WDT_KICK: LOW and left alone.
  pinMode(PIN_WDT_KICK, OUTPUT);
  digitalWrite(PIN_WDT_KICK, LOW);

  // 3. ESTOP_SENSE as input.
  pinMode(PIN_ESTOP_SENSE, INPUT);

  // 4. Last: ask for the drivers. Not done if LEDC failed, so a bad setup cannot energize anything.
  g_setup_ok = ok;
  if (ok) {
    pinMode(PIN_EN_MCU, OUTPUT);
    digitalWrite(PIN_EN_MCU, HIGH);
  }
  Serial.println(ok ? "T6b: PWM=0, WDT_KICK=LOW (no edges), EN_MCU=HIGH. Scope EN_DRV and WDT_OK."
                    : "T6b: LEDC init FAILED, EN_MCU left low. Test invalid.");
}

void loop() {
  static uint32_t n = 0;
  Serial.printf("t=%us EN_MCU=%d WDT_KICK=%d ESTOP_SENSE=%d setup_ok=%d\n", (unsigned)(++n),
                digitalRead(PIN_EN_MCU), digitalRead(PIN_WDT_KICK), digitalRead(PIN_ESTOP_SENSE), g_setup_ok);
  delay(1000);
}
