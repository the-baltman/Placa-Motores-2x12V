// PlacaProtocol - ESP-NOW command packet shared by receiver and transmitter.
// Pure C++ (no Arduino/ESP headers) so it is unit-tested on the PC (firmware/test).
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace placa {

// ---- Wire format (little-endian, explicit serialization, no struct memcpy) ----
//  0  magic    u8   0xB7
//  1  version  u8   1
//  2  session  u16  random per transmitter boot
//  4  seq      u16  +1 per packet, wraps
//  6  flags    u8   bit0 = CLEAR_FAULT; other bits must be 0
//  7  m1       i16  motor 1 command, permille (-1000..+1000), + = forward
//  9  m2       i16  motor 2 command, same units
// 11  crc16    u16  CRC-16/CCITT-FALSE over bytes 0..10
constexpr uint8_t PROTO_MAGIC = 0xB7;
constexpr uint8_t PROTO_VERSION = 1;
constexpr size_t CMD_PACKET_LEN = 13;
constexpr int16_t CMD_MAX_PERMILLE = 1000;
constexpr uint8_t FLAG_CLEAR_FAULT = 0x01;
constexpr uint8_t FLAGS_KNOWN_MASK = 0x01;

// Largest forward seq jump accepted while the link is up. Wrap-safe (signed 16-bit diff).
constexpr int16_t SEQ_MAX_JUMP = 256;

struct CmdPacket {
  uint16_t session;
  uint16_t seq;
  uint8_t flags;
  int16_t m1;
  int16_t m2;
};

enum class ParseResult : uint8_t {
  OK = 0,
  BAD_LENGTH,
  BAD_MAGIC,
  BAD_VERSION,
  BAD_CRC,
  BAD_FLAGS,
  BAD_RANGE,
};

uint16_t crc16_ccitt(const uint8_t* data, size_t len);

// Returns bytes written (CMD_PACKET_LEN) or 0 if cap is too small / command out of range.
size_t cmd_encode(const CmdPacket& p, uint8_t* out, size_t cap);

// Validates length, magic, version, CRC, reserved flags and command range.
ParseResult cmd_decode(const uint8_t* data, size_t len, CmdPacket& out);

// Age of the last packet. Signed difference clamped at 0: the callback (another core) can store
// last_rx_ms later than the `now` the control task captured earlier in the same cycle; an unsigned
// subtraction would wrap to ~4.29e9 and drop the link in normal operation.
inline uint32_t age_ms_clamped(uint32_t now_ms, uint32_t last_ms) {
  const int32_t d = (int32_t)(now_ms - last_ms);
  return d < 0 ? 0u : (uint32_t)d;
}

// Freshness filter: drops duplicates, reordered and replayed packets.
struct SeqFilter {
  bool have = false;
  uint16_t session = 0;
  uint16_t last_seq = 0;
};

// link_up = the receiver currently considers the link alive (packet within timeout).
// While the link is down any valid packet re-syncs (covers a transmitter reboot that
// restarts seq at 0). While it is up, a different session is rejected.
bool seq_accept(SeqFilter& f, uint16_t session, uint16_t seq, bool link_up);

}  // namespace placa
