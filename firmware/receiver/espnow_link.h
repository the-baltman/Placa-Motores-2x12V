// espnow_link.h - ESP-NOW receiver: validates packets (CRC, range, freshness, source MAC) and
// tracks link liveness. The receive callback only validates and stores; all decisions are made
// by the control task through espnow_link_poll().
#pragma once
#include <stdint.h>

struct LinkSnapshot {
  bool alive;          // a valid fresh packet arrived within the timeout
  bool new_packet;     // at least one valid packet since the previous poll
  int16_t m1, m2;      // latest command, permille; forced to 0 when !alive
  uint8_t flags;       // latest packet flags (placa::FLAG_*)
  uint32_t age_ms;     // time since the last valid packet (UINT32_MAX if none yet)
  bool ever_linked;    // a valid packet has been received at least once since boot
  uint32_t rx_ok, rx_bad, rx_old, rx_foreign;  // counters: valid, failed CRC/format, stale/duplicate, wrong source MAC
};

// WiFi STA, fixed channel, power save off, esp_now_init, callback. false if any call fails.
bool espnow_link_init();

// Runtime-configurable timeout; clamped to [LINK_TIMEOUT_MS_MIN, LINK_TIMEOUT_MS_MAX].
void espnow_link_set_timeout_ms(uint32_t ms);
uint32_t espnow_link_timeout_ms();

void espnow_link_poll(uint32_t now_ms, LinkSnapshot& out);

// Own station MAC as "AA:BB:CC:DD:EE:FF" (for the transmitter's peer config). buf >= 18 bytes.
void espnow_link_mac_string(char* buf, unsigned len);
