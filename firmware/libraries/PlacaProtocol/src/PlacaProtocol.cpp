#include "PlacaProtocol.h"

namespace placa {

uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static void put_u16(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)(v >> 8);
}
static uint16_t get_u16(const uint8_t* p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }

size_t cmd_encode(const CmdPacket& p, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < CMD_PACKET_LEN) return 0;
  if (p.m1 < -CMD_MAX_PERMILLE || p.m1 > CMD_MAX_PERMILLE) return 0;
  if (p.m2 < -CMD_MAX_PERMILLE || p.m2 > CMD_MAX_PERMILLE) return 0;
  if (p.flags & ~FLAGS_KNOWN_MASK) return 0;
  out[0] = PROTO_MAGIC;
  out[1] = PROTO_VERSION;
  put_u16(out + 2, p.session);
  put_u16(out + 4, p.seq);
  out[6] = p.flags;
  put_u16(out + 7, (uint16_t)p.m1);
  put_u16(out + 9, (uint16_t)p.m2);
  put_u16(out + 11, crc16_ccitt(out, 11));
  return CMD_PACKET_LEN;
}

ParseResult cmd_decode(const uint8_t* data, size_t len, CmdPacket& out) {
  if (data == nullptr || len != CMD_PACKET_LEN) return ParseResult::BAD_LENGTH;
  if (data[0] != PROTO_MAGIC) return ParseResult::BAD_MAGIC;
  if (data[1] != PROTO_VERSION) return ParseResult::BAD_VERSION;
  if (get_u16(data + 11) != crc16_ccitt(data, 11)) return ParseResult::BAD_CRC;
  if (data[6] & ~FLAGS_KNOWN_MASK) return ParseResult::BAD_FLAGS;
  const int16_t m1 = (int16_t)get_u16(data + 7);
  const int16_t m2 = (int16_t)get_u16(data + 9);
  if (m1 < -CMD_MAX_PERMILLE || m1 > CMD_MAX_PERMILLE) return ParseResult::BAD_RANGE;
  if (m2 < -CMD_MAX_PERMILLE || m2 > CMD_MAX_PERMILLE) return ParseResult::BAD_RANGE;
  out.session = get_u16(data + 2);
  out.seq = get_u16(data + 4);
  out.flags = data[6];
  out.m1 = m1;
  out.m2 = m2;
  return ParseResult::OK;
}

bool seq_accept(SeqFilter& f, uint16_t session, uint16_t seq, bool link_up) {
  if (!f.have || !link_up) {
    f.have = true;
    f.session = session;
    f.last_seq = seq;
    return true;
  }
  if (session != f.session) return false;
  const int16_t diff = (int16_t)(uint16_t)(seq - f.last_seq);
  if (diff <= 0 || diff > SEQ_MAX_JUMP) return false;
  f.last_seq = seq;
  return true;
}

}  // namespace placa
