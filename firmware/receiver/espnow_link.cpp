#include "espnow_link.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>

#include <PlacaProtocol.h>

#include "config.h"

namespace {

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

// Shared between the WiFi task (callback) and the control task; guarded by g_mux.
struct Shared {
  placa::SeqFilter filter;
  bool have_rx = false;
  uint32_t last_rx_ms = 0;
  uint32_t pkt_count = 0;  // valid packets since boot
  int16_t m1 = 0, m2 = 0;
  uint8_t flags = 0;
  uint32_t rx_ok = 0, rx_bad = 0, rx_old = 0, rx_foreign = 0;
};
Shared g_sh;
uint32_t g_timeout_ms = cfg::LINK_TIMEOUT_MS_DEFAULT;
uint32_t g_last_polled_count = 0;

bool mac_filter_enabled() {
  for (uint8_t b : cfg::TX_MAC_FILTER) {
    if (b != 0) return true;
  }
  return false;
}

// Runs in the WiFi task: short, no blocking, no logging.
void on_recv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  const uint32_t now = millis();
  if (info == nullptr || data == nullptr || len < 0) return;

  if (mac_filter_enabled() && memcmp(info->src_addr, cfg::TX_MAC_FILTER, 6) != 0) {
    portENTER_CRITICAL(&g_mux);
    g_sh.rx_foreign++;
    portEXIT_CRITICAL(&g_mux);
    return;
  }

  placa::CmdPacket p;
  const placa::ParseResult r = placa::cmd_decode(data, (size_t)len, p);

  portENTER_CRITICAL(&g_mux);
  if (r != placa::ParseResult::OK) {
    g_sh.rx_bad++;
  } else {
    const bool link_up = g_sh.have_rx && placa::age_ms_clamped(now, g_sh.last_rx_ms) <= g_timeout_ms;
    if (placa::seq_accept(g_sh.filter, p.session, p.seq, link_up)) {
      g_sh.have_rx = true;
      g_sh.last_rx_ms = now;
      g_sh.m1 = p.m1;
      g_sh.m2 = p.m2;
      g_sh.flags = p.flags;
      g_sh.pkt_count++;
      g_sh.rx_ok++;
    } else {
      g_sh.rx_old++;
    }
  }
  portEXIT_CRITICAL(&g_mux);
}

}  // namespace

bool espnow_link_init() {
  if (!WiFi.mode(WIFI_STA)) return false;
  WiFi.disconnect();
  if (esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK) return false;  // power save adds rx latency
  if (esp_wifi_set_channel(cfg::ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) return false;
  if (esp_now_init() != ESP_OK) return false;
  if (esp_now_register_recv_cb(on_recv) != ESP_OK) return false;
  return true;
}

void espnow_link_set_timeout_ms(uint32_t ms) {
  if (ms < cfg::LINK_TIMEOUT_MS_MIN) ms = cfg::LINK_TIMEOUT_MS_MIN;
  if (ms > cfg::LINK_TIMEOUT_MS_MAX) ms = cfg::LINK_TIMEOUT_MS_MAX;
  portENTER_CRITICAL(&g_mux);
  g_timeout_ms = ms;
  portEXIT_CRITICAL(&g_mux);
}

uint32_t espnow_link_timeout_ms() { return g_timeout_ms; }

void espnow_link_poll(uint32_t now_ms, LinkSnapshot& out) {
  Shared s;
  uint32_t timeout;
  portENTER_CRITICAL(&g_mux);
  s = g_sh;
  timeout = g_timeout_ms;
  portEXIT_CRITICAL(&g_mux);

  out.ever_linked = s.have_rx;
  out.age_ms = s.have_rx ? placa::age_ms_clamped(now_ms, s.last_rx_ms) : UINT32_MAX;
  out.alive = s.have_rx && out.age_ms <= timeout;
  out.new_packet = s.pkt_count != g_last_polled_count;
  g_last_polled_count = s.pkt_count;
  out.m1 = out.alive ? s.m1 : 0;
  out.m2 = out.alive ? s.m2 : 0;
  out.flags = out.alive ? s.flags : 0;
  out.rx_ok = s.rx_ok;
  out.rx_bad = s.rx_bad;
  out.rx_old = s.rx_old;
  out.rx_foreign = s.rx_foreign;
}

void espnow_link_mac_string(char* buf, unsigned len) {
  if (buf == nullptr || len < 18) return;
  uint8_t mac[6] = {0};
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  snprintf(buf, len, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}
