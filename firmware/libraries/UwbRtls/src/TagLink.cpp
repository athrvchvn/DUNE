#include "TagLink.h"
#include <esp_timer.h>

// UART2 via the GPIO matrix (any output-capable pin for TX; 35 is input-only,
// which is fine for RX). Serial2 is otherwise unused in this project.
#define TL_UART Serial2

// Extra wait beyond the granted budget for the DATA/DONE frames themselves
// plus the slave's loop latency (~100 B at 921600 baud is ~1.1 ms on wire).
static const uint16_t TL_DONE_MARGIN_MS = 50;

// Wire payload layouts (packed; both ends are little-endian ESP32).
struct __attribute__((packed)) TlHello   { uint32_t buildId; uint8_t tagId; uint8_t caps; };
struct __attribute__((packed)) TlGo      { uint16_t cycleId; uint16_t budgetMs; };
struct __attribute__((packed)) TlDataHdr { uint16_t cycleId; uint32_t slaveSeq;
                                           uint32_t tMs; uint8_t tagId;
                                           uint32_t sweepUs; uint8_t n; };
struct __attribute__((packed)) TlDone    { uint16_t cycleId; uint32_t slaveSeq; };

// ---------------------------------------------------------------------------

uint16_t TagLink::crc16(uint16_t crc, uint8_t b) {
  // CRC-16/CCITT-FALSE, bitwise (tiny; frames are ~100 B at most).
  crc ^= (uint16_t)b << 8;
  for (uint8_t i = 0; i < 8; i++)
    crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  return crc;
}

uint32_t TagLink::buildId() {
  // FNV-1a over the compile timestamp: differing values on the two boards are
  // expected (separate compiles) and only WARN — same source, different build.
  const char* s = __DATE__ " " __TIME__;
  uint32_t h = 2166136261u;
  for (; *s; s++) { h ^= (uint8_t)*s; h *= 16777619u; }
  return h;
}

void TagLink::begin(TagRole role, uint8_t tagId, uint8_t caps) {
  _role  = role;
  _tagId = tagId;
  _caps  = caps;
  if (_role == TAG_ROLE_SOLO) return;   // inert: no UART claimed

  TL_UART.setRxBufferSize(512);
  TL_UART.begin(TAGLINK_BAUD, SERIAL_8N1, TAGLINK_PIN_RX, TAGLINK_PIN_TX);
  _lastHelloUs = esp_timer_get_time();
  Serial.printf("[LINK] %s tag 0x%02X, UART2 TX=%d RX=%d @%d, proto v%d build %08lX\n",
                _role == TAG_ROLE_MASTER ? "MASTER" : "SLAVE", _tagId,
                TAGLINK_PIN_TX, TAGLINK_PIN_RX, (int)TAGLINK_BAUD,
                TAGLINK_PROTO_VER, (unsigned long)buildId());
}

// ---------------------------------------------------------------------------
// Framing
// ---------------------------------------------------------------------------

void TagLink::sendFrame(uint8_t type, const uint8_t* payload, uint8_t len) {
  uint8_t hdr[4] = { 0xA5, TAGLINK_PROTO_VER, type, len };
  uint16_t crc = 0xFFFF;
  crc = crc16(crc, hdr[1]); crc = crc16(crc, hdr[2]); crc = crc16(crc, hdr[3]);
  for (uint8_t i = 0; i < len; i++) crc = crc16(crc, payload[i]);
  TL_UART.write(hdr, 4);
  if (len) TL_UART.write(payload, len);
  uint8_t tail[2] = { (uint8_t)(crc & 0xFF), (uint8_t)(crc >> 8) };
  TL_UART.write(tail, 2);
}

void TagLink::feedByte(uint8_t b) {
  switch (_rxState) {
    case RX_SOF:
      if (b == 0xA5) _rxState = RX_VER;
      else _stats.resync++;
      break;
    case RX_VER:
      if (b == TAGLINK_PROTO_VER) { _rxCrc = crc16(0xFFFF, b); _rxState = RX_TYPE; }
      else { _stats.resync++; _rxState = RX_SOF; }   // mixed firmware: reject here
      break;
    case RX_TYPE:
      _rxType = b; _rxCrc = crc16(_rxCrc, b); _rxState = RX_LEN;
      break;
    case RX_LEN:
      _rxLen = b; _rxGot = 0; _rxCrc = crc16(_rxCrc, b);
      _rxState = _rxLen ? RX_PAYLOAD : RX_CRC_LO;
      break;
    case RX_PAYLOAD:
      _rxBuf[_rxGot++] = b; _rxCrc = crc16(_rxCrc, b);
      if (_rxGot >= _rxLen) _rxState = RX_CRC_LO;
      break;
    case RX_CRC_LO:
      _rxCrcLo = b;
      _rxState = RX_CRC_HI;
      break;
    case RX_CRC_HI: {
      uint16_t rxCrc = (uint16_t)_rxCrcLo | ((uint16_t)b << 8);
      if (rxCrc == _rxCrc) handleFrame(_rxType, _rxBuf, _rxLen);
      else _stats.crcErr++;
      _rxState = RX_SOF;
      break;
    }
  }
}

void TagLink::poll() {
  if (_role == TAG_ROLE_SOLO) return;
  while (TL_UART.available()) feedByte((uint8_t)TL_UART.read());
  if (_role == TAG_ROLE_SLAVE) slaveHousekeeping();
}

// ---------------------------------------------------------------------------
// Frame dispatch
// ---------------------------------------------------------------------------

void TagLink::handleFrame(uint8_t type, const uint8_t* p, uint8_t len) {
  switch (type) {

    case TL_HELLO: {                                    // master receives
      if (_role != TAG_ROLE_MASTER || len < sizeof(TlHello)) return;
      TlHello h; memcpy(&h, p, sizeof(h));
      if (!_slavePresent)
        Serial.printf("[LINK] slave 0x%02X joined (build %08lX, caps %02X)\n",
                      h.tagId, (unsigned long)h.buildId, h.caps);
      if (h.buildId != buildId())
        Serial.printf("[LINK] WARNING: build mismatch (master %08lX / slave %08lX)"
                      " — same source? reflash both together\n",
                      (unsigned long)buildId(), (unsigned long)h.buildId);
      _peerTagId    = h.tagId;
      _peerBuildId  = h.buildId;
      _peerCaps     = h.caps;
      _slavePresent = true;
      _timeoutStreak = 0;
      TlHello ack = { buildId(), _tagId, _caps };
      sendFrame(TL_HELLO_ACK, (const uint8_t*)&ack, sizeof(ack));
      break;
    }

    case TL_HELLO_ACK:                                  // slave receives
      if (_role != TAG_ROLE_SLAVE) return;
      if (!_helloAcked) Serial.println("[LINK] master acknowledged — joined");
      _helloAcked = true;
      break;

    case TL_GO: {                                       // slave receives
      if (_role != TAG_ROLE_SLAVE || len < sizeof(TlGo)) return;
      TlGo g; memcpy(&g, p, sizeof(g));
      _goPending  = true;
      _goCycle    = g.cycleId;
      _goBudgetMs = g.budgetMs;
      _helloAcked = true;   // a GO implies the master knows us
      break;
    }

    case TL_DATA: {                                     // master receives
      if (_role != TAG_ROLE_MASTER || len < sizeof(TlDataHdr)) return;
      TlDataHdr h; memcpy(&h, p, sizeof(h));
      if (h.cycleId != _cycleId) { _stats.staleCycle++; return; }
      if (_seqSeen) {
        if (h.slaveSeq == _lastSlaveSeq)     _stats.seqDup++;
        else if (h.slaveSeq != _lastSlaveSeq + 1) _stats.seqGap++;
      }
      _lastSlaveSeq = h.slaveSeq;
      _seqSeen = true;

      uint8_t n = h.n;
      if (n > TAGLINK_MAX_RECORDS) n = TAGLINK_MAX_RECORDS;
      if (len < sizeof(TlDataHdr) + n * sizeof(TagLinkRange)) return;
      const uint8_t* rec = p + sizeof(TlDataHdr);
      for (uint8_t i = 0; i < n; i++) {
        TagLinkRange r; memcpy(&r, rec + i * sizeof(TagLinkRange), sizeof(r));
        _dataRanges[i].id       = r.id;
        _dataRanges[i].valid    = true;   // slave only ships valid results
        _dataRanges[i].distance = r.distance;
        _dataRanges[i].rxPower  = r.rxPower;
        _dataRanges[i].fpPower  = r.fpPower;
        _dataRanges[i].quality  = r.quality;
      }
      _dataCount   = n;
      _dataTagMs   = h.tMs;
      _dataTagId   = h.tagId;
      _dataSweepUs = h.sweepUs;
      _dataRxUs    = esp_timer_get_time();   // MRX: master clock at arrival
      _dataValid   = true;
      updateBudget(h.sweepUs);
      break;
    }

    case TL_DONE: {                                     // master receives
      if (_role != TAG_ROLE_MASTER || len < sizeof(TlDone)) return;
      TlDone d; memcpy(&d, p, sizeof(d));
      if (d.cycleId != _cycleId) { _stats.staleCycle++; return; }
      _doneSeen = true;
      break;
    }
  }
}

// ---------------------------------------------------------------------------
// Master role
// ---------------------------------------------------------------------------

uint16_t TagLink::startCycle() {
  _cycleId++;
  _dataValid = false;
  _doneSeen  = false;
  return _cycleId;
}

void TagLink::grantSlot() {
  TlGo g = { _cycleId, _budgetMs };
  sendFrame(TL_GO, (const uint8_t*)&g, sizeof(g));
}

bool TagLink::waitDone(void (*idleFn)()) {
  int64_t t0       = esp_timer_get_time();
  int64_t deadline = t0 + (int64_t)(_budgetMs + TL_DONE_MARGIN_MS) * 1000;
  while (!_doneSeen) {
    poll();
    if (idleFn) idleFn();
    if (esp_timer_get_time() > deadline) {
      _uartWaitUs = (uint32_t)(esp_timer_get_time() - t0);
      _stats.slotTimeout++;
      if (++_timeoutStreak >= TAGLINK_ABSENT_AFTER && _slavePresent) {
        _slavePresent = false;
        Serial.printf("[LINK] slave absent after %u slot timeouts — running solo"
                      " (rejoins on HELLO)\n", _timeoutStreak);
      }
      return false;
    }
    yield();
  }
  _uartWaitUs   = (uint32_t)(esp_timer_get_time() - t0);
  _timeoutStreak = 0;
  return true;
}

void TagLink::updateBudget(uint32_t sweepUs) {
  // Grow instantly on a slower sweep, shrink slowly (alpha=0.1) — a lucky fast
  // sweep must never starve a legitimate worst-case one into "absent".
  if ((float)sweepUs > _sweepEmaUs) _sweepEmaUs = (float)sweepUs;
  else                              _sweepEmaUs += 0.1f * ((float)sweepUs - _sweepEmaUs);
  uint32_t ms = (uint32_t)(1.5f * _sweepEmaUs / 1000.0f);
  if (ms < TAGLINK_BUDGET_FLOOR_MS) ms = TAGLINK_BUDGET_FLOOR_MS;
  if (ms > TAGLINK_BUDGET_CEIL_MS)  ms = TAGLINK_BUDGET_CEIL_MS;
  _budgetMs = (uint16_t)ms;
}

// ---------------------------------------------------------------------------
// Slave role
// ---------------------------------------------------------------------------

void TagLink::slaveHousekeeping() {
  int64_t now = esp_timer_get_time();
  int64_t period = 1000LL * (_helloAcked ? TAGLINK_HELLO_BEAT_MS
                                         : TAGLINK_HELLO_FAST_MS);
  if (now - _lastHelloUs < period) return;
  _lastHelloUs = now;
  TlHello h = { buildId(), _tagId, _caps };
  sendFrame(TL_HELLO, (const uint8_t*)&h, sizeof(h));
}

bool TagLink::pollGo(uint16_t& cycleId, uint16_t& budgetMs) {
  if (!_goPending) return false;
  _goPending = false;
  cycleId  = _goCycle;
  budgetMs = _goBudgetMs;
  _cycleId = _goCycle;    // mirror the master's cycle for DATA/DONE
  return true;
}

void TagLink::sendData(uint32_t tMs, uint8_t tagId, uint32_t sweepUs,
                       const RangeResult* results, uint8_t nAll) {
  // Only valid entries go on the wire; the master rebuilds RangeResults and
  // feeds them to the same HostLink serializer used for its own sweep.
  uint8_t buf[sizeof(TlDataHdr) + TAGLINK_MAX_RECORDS * sizeof(TagLinkRange)];
  uint8_t n = 0;
  for (uint8_t i = 0; i < nAll && n < TAGLINK_MAX_RECORDS; i++) {
    if (!results[i].valid) continue;
    TagLinkRange r = { results[i].id, results[i].distance, results[i].rxPower,
                       results[i].fpPower, results[i].quality };
    memcpy(buf + sizeof(TlDataHdr) + n * sizeof(TagLinkRange), &r, sizeof(r));
    n++;
  }
  TlDataHdr h = { _cycleId, ++_slaveSeq, tMs, tagId, sweepUs, n };
  memcpy(buf, &h, sizeof(h));
  sendFrame(TL_DATA, buf, sizeof(TlDataHdr) + n * sizeof(TagLinkRange));
}

void TagLink::sendDone() {
  TlDone d = { _cycleId, _slaveSeq };
  sendFrame(TL_DONE, (const uint8_t*)&d, sizeof(d));
}
