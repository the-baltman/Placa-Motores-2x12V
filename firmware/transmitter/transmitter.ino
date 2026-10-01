// transmitter.ino - test transmitter for Placa-Motores-2x12V (any ESP32, Arduino-ESP32 core 3.x).
// Sends one command packet every 20 ms (50 Hz). Two sources:
//   SERIAL (default): type  "m <m1> <m2>"  (permille -1000..1000), "s" = stop, "c" = send CLEAR_FAULT for 500 ms.
//                     A serial command expires after SERIAL_HOLD_MS (dead-man): then the transmitter sends 0.
//   JOYSTICK (USE_JOYSTICK = 1): analog stick on JOY_X_PIN / JOY_Y_PIN, arcade mixing, button = CLEAR_FAULT.
// Unplugging this transmitter must stop the motors within the receiver's link timeout; test that.
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_mac.h>
#include <esp_random.h>
#include <esp_wifi.h>

#include <PlacaProtocol.h>

// ---- settings
#define USE_JOYSTICK 0
static const uint8_t ESPNOW_CHANNEL = 1;                                  // must equal receiver cfg::ESPNOW_CHANNEL
static const uint8_t RX_MAC[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};    // MANDATORY: receiver MAC printed at its boot (unicast only)
static const uint32_t SEND_PERIOD_MS = 20;                                // 50 Hz
static const uint32_t SERIAL_HOLD_MS = 500;                               // serial command validity
static const uint32_t CLEAR_FLAG_MS = 500;                                // CLEAR_FAULT duration
// Joystick wiring (ADC1 pins, on the TRANSMITTER board): centre ~ 1.65 V.
static const uint8_t JOY_X_PIN = 34;      // steering
static const uint8_t JOY_Y_PIN = 35;      // throttle
static const uint8_t JOY_BTN_PIN = 32;    // active low to GND, internal pull-up
static const int JOY_CENTER_MV = 1650;
static const int JOY_RANGE_MV = 1400;     // mV from centre to full deflection
static const int JOY_DEADZONE_PERMILLE = 60;

static uint16_t g_session = 0;
static uint16_t g_seq = 0;
static uint32_t g_send_ok = 0, g_send_fail = 0, g_cb_fail = 0;
static int16_t g_m1 = 0, g_m2 = 0;
static uint32_t g_cmd_ms = 0, g_clear_until_ms = 0;

static void on_sent(const esp_now_send_info_t*, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) g_cb_fail++;  // unicast: FAIL = no ACK from the receiver (out of range, wrong channel or MAC)
}

static int16_t clamp_pm(long v) { return (int16_t)(v > 1000 ? 1000 : (v < -1000 ? -1000 : v)); }

static void read_serial() {
  static char buf[32];
  static uint8_t n = 0;
  while (Serial.available() > 0) {
    const char ch = (char)Serial.read();
    if (ch != '\n' && ch != '\r') {
      if (n < sizeof(buf) - 1) buf[n++] = ch;
      continue;
    }
    buf[n] = 0;
    n = 0;
    long a = 0, b = 0;
    if (buf[0] == 's') {
      g_m1 = g_m2 = 0;
      g_cmd_ms = millis();
    } else if (buf[0] == 'c') {
      g_clear_until_ms = millis() + CLEAR_FLAG_MS;
    } else if (buf[0] == 'm' && sscanf(buf + 1, "%ld %ld", &a, &b) == 2) {
      g_m1 = clamp_pm(a);
      g_m2 = clamp_pm(b);
      g_cmd_ms = millis();
    }
  }
}

#if USE_JOYSTICK
static int16_t axis_pm(uint8_t pin) {
  const long mv = analogReadMilliVolts(pin);
  long v = ((mv - JOY_CENTER_MV) * 1000L) / JOY_RANGE_MV;
  if (v > -JOY_DEADZONE_PERMILLE && v < JOY_DEADZONE_PERMILLE) return 0;
  return clamp_pm(v);
}
static void read_joystick() {
  const int16_t thr = axis_pm(JOY_Y_PIN), steer = axis_pm(JOY_X_PIN);
  g_m1 = clamp_pm((long)thr + steer);
  g_m2 = clamp_pm((long)thr - steer);
  g_cmd_ms = millis();
  if (digitalRead(JOY_BTN_PIN) == LOW) g_clear_until_ms = millis() + CLEAR_FLAG_MS;
}
#endif

void setup() {
  Serial.begin(115200);
  g_session = (uint16_t)(esp_random() & 0xFFFF);
  uint8_t my_mac[6] = {0};
  const bool mac_read = esp_read_mac(my_mac, ESP_MAC_WIFI_STA) == ESP_OK;
  if (mac_read) {  // needed for receiver cfg::TX_MAC_FILTER; printed before any halt
    Serial.printf("TX MAC %02X:%02X:%02X:%02X:%02X:%02X\n", my_mac[0], my_mac[1], my_mac[2], my_mac[3], my_mac[4], my_mac[5]);
  } else {
    Serial.println("WARN: esp_read_mac failed");
  }
  bool mac_set = false;
  for (uint8_t b : RX_MAC) mac_set = mac_set || (b != 0);
  if (!mac_set) {
    Serial.println("\nERROR: set RX_MAC to the receiver's MAC (printed at its boot). Unicast pairing is mandatory.");
    for (;;) delay(1000);
  }
  bool ok = WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  ok = ok && esp_wifi_set_ps(WIFI_PS_NONE) == ESP_OK;
  ok = ok && esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) == ESP_OK;
  ok = ok && esp_now_init() == ESP_OK;
  ok = ok && esp_now_register_send_cb(on_sent) == ESP_OK;
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, RX_MAC, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  ok = ok && esp_now_add_peer(&peer) == ESP_OK;
#if USE_JOYSTICK
  pinMode(JOY_BTN_PIN, INPUT_PULLUP);
  analogSetPinAttenuation(JOY_X_PIN, ADC_11db);
  analogSetPinAttenuation(JOY_Y_PIN, ADC_11db);
#endif
  Serial.printf("transmitter %s, session 0x%04X, channel %u\n", ok ? "ready" : "INIT FAILED", g_session, ESPNOW_CHANNEL);
  if (!ok) {
    for (;;) delay(1000);  // nothing to transmit without a radio; no actuator on this board
  }
}

void loop() {
  static TickType_t last = xTaskGetTickCount();
  xTaskDelayUntil(&last, pdMS_TO_TICKS(SEND_PERIOD_MS));
  const uint32_t now = millis();

#if USE_JOYSTICK
  read_joystick();
#else
  read_serial();
  if ((uint32_t)(now - g_cmd_ms) > SERIAL_HOLD_MS) g_m1 = g_m2 = 0;
#endif

  placa::CmdPacket p;
  p.session = g_session;
  p.seq = ++g_seq;
  p.flags = ((int32_t)(g_clear_until_ms - now) > 0) ? placa::FLAG_CLEAR_FAULT : 0;
  p.m1 = g_m1;
  p.m2 = g_m2;
  uint8_t buf[placa::CMD_PACKET_LEN];
  const size_t n = placa::cmd_encode(p, buf, sizeof(buf));
  if (n == 0 || esp_now_send(RX_MAC, buf, n) != ESP_OK) {
    g_send_fail++;
  } else {
    g_send_ok++;
  }

  static uint32_t last_print = 0;
  if (now - last_print >= 1000) {
    last_print = now;
    Serial.printf("seq=%u m1=%d m2=%d flags=%u sent=%u fail=%u cb_fail=%u\n", p.seq, p.m1, p.m2, p.flags,
                  (unsigned)g_send_ok, (unsigned)g_send_fail, (unsigned)g_cb_fail);
  }
}
