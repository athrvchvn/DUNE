/*
 * TagLink.h - Wired master/slave TDMA coordination for dual-tag operation.
 *
 * Replaces the retired over-air token ring (TagRing). The two tags are rigidly
 * mounted ~50 cm apart, so coordination runs over a dedicated UART instead of
 * the lossy shared UWB channel: exactly one tag's radio is active at a time,
 * and ownership of the air is decided by wire, never by radio.
 *
 *   MASTER (has the IMU, publishes to the host over UDP):
 *     startCycle() -> sweep -> setRadioState(RADIO_IDLE) -> grantSlot() ->
 *     waitDone(idleFn)   // IMU / UDP / OLED run as idleFn WHILE slave sweeps
 *     -> forward slave data -> next cycle.
 *   SLAVE (no WiFi, radio idle except its slot):
 *     pollGo() -> setRadioState(RADIO_ACTIVE) -> sweep ->
 *     setRadioState(RADIO_IDLE) -> sendData() -> sendDone().
 *
 * Design philosophy: the slave's slot is NOT master wait time — it is master
 * CPU time to exploit. GO is issued the moment the master's own sweep ends and
 * all master-side processing overlaps the slave's sweep via waitDone's idleFn.
 *
 * Wire framing (binary; ASCII exists only on the host link):
 *   0xA5 | proto_ver | type | len | payload[len] | crc16-ccitt(ver,type,len,payload)
 * The protocol version rides in EVERY frame so mismatched firmware is rejected
 * at the parser, not just at HELLO. The byte-stream parser resyncs on SOF and
 * keeps link-health counters (crc/resync/stale-cycle/seq gap+dup) so a bad
 * cable is visible instantly.
 *
 * Timing uses esp_timer_get_time() (µs) throughout — never millis().
 * Cycle number is the scheduler's primary key: every message carries the
 * master's cycle_id; DATA additionally carries an independent u32 slave_seq so
 * UART duplication/loss is distinguishable from cycle mismatch.
 */
#ifndef UWBRTLS_TAGLINK_H
#define UWBRTLS_TAGLINK_H

#include <Arduino.h>
#include "UwbConfig.h"
#include "UwbScheduler.h"   // RangeResult

// Protocol version — bump on ANY wire-format change.
#define TAGLINK_PROTO_VER  1

// Capability bits advertised in HELLO (bit set = peripheral present).
#define TAGLINK_CAP_IMU      0x01
#define TAGLINK_CAP_BATTERY  0x02
// bits 2..7 reserved

enum TagRole : uint8_t { TAG_ROLE_MASTER, TAG_ROLE_SLAVE, TAG_ROLE_SOLO };

// Most anchors' results one DATA frame can carry (u8 payload length bound).
#define TAGLINK_MAX_RECORDS  12

// Link-health counters (both ends keep their own; master surfaces them).
struct TagLinkStats {
  uint32_t crcErr     = 0;  // frames dropped on CRC mismatch (bad cable!)
  uint32_t resync     = 0;  // bytes skipped hunting for SOF / bad version
  uint32_t staleCycle = 0;  // frames whose cycle_id != current cycle
  uint32_t seqGap     = 0;  // slave_seq jumped forward (UART loss)
  uint32_t seqDup     = 0;  // slave_seq repeated (UART duplication)
  uint32_t slotTimeout= 0;  // master: slave slots that never DONEd
};

// Packed wire records (both ends are ESP32 little-endian; packed for determinism).
struct __attribute__((packed)) TagLinkRange {
  uint8_t id;
  float   distance;   // metres
  float   rxPower;    // dBm
  float   fpPower;    // dBm
  float   quality;
};

class TagLink {
public:
  // role: MASTER or SLAVE (SOLO = inert, every call is a cheap no-op).
  // tagId: this board's UWB short address. caps: TAGLINK_CAP_* bits.
  void begin(TagRole role, uint8_t tagId, uint8_t caps = 0);

  // Drain the UART and update protocol state. Call every loop() iteration;
  // also called internally by waitDone(). Non-blocking.
  void poll();

  TagRole             role()  const { return _role; }
  const TagLinkStats& stats() const { return _stats; }

  // ---- MASTER -------------------------------------------------------------
  bool     slavePresent() const { return _slavePresent; }
  uint8_t  slaveTagId()   const { return _peerTagId; }
  uint32_t slaveBuildId() const { return _peerBuildId; }
  uint8_t  slaveCaps()    const { return _peerCaps; }

  // Advance to the next cycle (increments cycle_id) and return it.
  uint16_t startCycle();

  // Grant the slave its air slot for the current cycle (sends GO with the
  // current adaptive budget). Call only when slavePresent().
  void grantSlot();

  // Block until the slave's DONE for this cycle, calling idleFn() repeatedly
  // (master's useful work overlaps the slave's sweep). Returns false on
  // timeout (budget + wire margin), which counts toward absence.
  bool waitDone(void (*idleFn)() = nullptr);

  // Slave data received during the last waitDone() (valid until next cycle).
  bool               hasData()      const { return _dataValid; }
  uint8_t            dataCount()    const { return _dataCount; }
  const RangeResult* dataRanges()   const { return _dataRanges; }
  uint32_t           dataTagMs()    const { return _dataTagMs; }    // slave clock, untouched
  uint8_t            dataTagId()    const { return _dataTagId; }
  uint32_t           dataSweepUs()  const { return _dataSweepUs; }
  int64_t            dataRxTimeUs() const { return _dataRxUs; }     // MRX (master clock)
  uint32_t           lastUartWaitUs() const { return _uartWaitUs; } // last waitDone duration
  uint16_t           currentBudgetMs() const { return _budgetMs; }
  uint16_t           cycleId()      const { return _cycleId; }

  // ---- SLAVE --------------------------------------------------------------
  // True exactly once per received GO; outputs the granted cycle and budget.
  bool pollGo(uint16_t& cycleId, uint16_t& budgetMs);

  // Send this sweep's results (only valid entries are put on the wire) and
  // the explicit end-of-slot marker.
  void sendData(uint32_t tMs, uint8_t tagId, uint32_t sweepUs,
                const RangeResult* results, uint8_t nAll);
  void sendDone();

  bool masterSeen() const { return _helloAcked; }

private:
  enum MsgType : uint8_t {
    TL_HELLO = 1, TL_HELLO_ACK = 2, TL_GO = 3, TL_DATA = 4, TL_DONE = 5,
  };

  void sendFrame(uint8_t type, const uint8_t* payload, uint8_t len);
  void handleFrame(uint8_t type, const uint8_t* payload, uint8_t len);
  void feedByte(uint8_t b);
  void slaveHousekeeping();     // HELLO cadence (fast until ACKed, then beat)
  void updateBudget(uint32_t sweepUs);
  static uint16_t crc16(uint16_t crc, uint8_t b);
  static uint32_t buildId();    // FNV-1a of __DATE__ __TIME__

  TagRole  _role   = TAG_ROLE_SOLO;
  uint8_t  _tagId  = 0;
  uint8_t  _caps   = 0;

  // RX parser state machine.
  enum RxState : uint8_t { RX_SOF, RX_VER, RX_TYPE, RX_LEN, RX_PAYLOAD, RX_CRC_LO, RX_CRC_HI };
  RxState  _rxState = RX_SOF;
  uint8_t  _rxType = 0, _rxLen = 0, _rxGot = 0, _rxCrcLo = 0;
  uint8_t  _rxBuf[255];
  uint16_t _rxCrc = 0;

  TagLinkStats _stats;

  // Cycle / scheduling (master owns _cycleId; slave mirrors the granted one).
  uint16_t _cycleId  = 0;
  uint16_t _budgetMs = TAGLINK_BUDGET_INIT_MS;
  float    _sweepEmaUs = 0.0f;

  // Master-side peer state.
  bool     _slavePresent = false;
  uint8_t  _timeoutStreak = 0;
  uint8_t  _peerTagId = 0, _peerCaps = 0;
  uint32_t _peerBuildId = 0;
  bool     _doneSeen = false;

  // Master-side received DATA (decoded back into RangeResult for HostLink).
  bool        _dataValid = false;
  uint8_t     _dataCount = 0;
  RangeResult _dataRanges[TAGLINK_MAX_RECORDS];
  uint32_t    _dataTagMs = 0, _dataSweepUs = 0;
  uint8_t     _dataTagId = 0;
  int64_t     _dataRxUs  = 0;
  uint32_t    _uartWaitUs = 0;
  uint32_t    _lastSlaveSeq = 0;
  bool        _seqSeen = false;

  // Slave-side state.
  bool     _helloAcked = false;
  int64_t  _lastHelloUs = 0;
  bool     _goPending = false;
  uint16_t _goCycle = 0, _goBudgetMs = 0;
  uint32_t _slaveSeq = 0;
};

#endif // UWBRTLS_TAGLINK_H
