/*
 * UwbScheduler.h - Tag-side round-robin TDMA over an arbitrary anchor list.
 *
 * This is the piece that defeats the 4-anchor wall: the tag holds an explicit
 * list of anchor short addresses and ranges to each in turn (one exchange at a
 * time, so anchors never collide). Adding an anchor = add one entry to the
 * list; nothing else in the firmware changes.
 *
 * Designed to extend to multiple tags later (a tag-level superframe keyed by
 * TAG_ADDR), but for now there is one tag.
 */
#ifndef UWBRTLS_UWBSCHEDULER_H
#define UWBRTLS_UWBSCHEDULER_H

#include <Arduino.h>
#include "TwrEngine.h"

#ifndef UWB_MAX_ANCHORS
#define UWB_MAX_ANCHORS 16    // bump if you need more; bounded only by RAM/time
#endif

struct RangeResult {
  uint8_t id       = UWB_ADDR_INVALID;
  bool    valid    = false;   // did this anchor answer this sweep?
  float   distance = 0.0f;    // metres
  float   rxPower  = 0.0f;    // dBm
  float   fpPower  = 0.0f;    // first-path power, dBm
  float   quality  = 0.0f;    // receive quality (FP amplitude / noise estimate)
};

class UwbScheduler {
public:
  // anchorAddrs: array of anchor short addresses; n: how many.
  void begin(TwrEngine* engine, const uint8_t* anchorAddrs, uint8_t n);

  // Range to every configured anchor once. Returns the number that answered.
  uint8_t sweep();

  uint8_t            anchorCount() const { return _n; }
  const RangeResult& result(uint8_t i) const { return _results[i]; }
  uint32_t           sweepSeq() const { return _sweepSeq; }

private:
  TwrEngine*  _engine = nullptr;
  uint8_t     _addrs[UWB_MAX_ANCHORS];
  RangeResult _results[UWB_MAX_ANCHORS];
  uint8_t     _n = 0;
  uint32_t    _sweepSeq = 0;

  // Dead-anchor skip / exponential backoff.
  // After SKIP_AFTER_FAILS consecutive failures the anchor is skipped for
  // SKIP_BASE_SWEEPS sweeps (doubling each time, capped at SKIP_MAX_SWEEPS).
  // The slot is retried after the skip window; backoff resets on any success.
  static constexpr uint8_t SKIP_AFTER_FAILS = 2;
  static constexpr uint8_t SKIP_BASE_SWEEPS = 2;
  static constexpr uint8_t SKIP_MAX_SWEEPS  = 40;

  uint8_t _failStreak[UWB_MAX_ANCHORS];   // consecutive rangeTo() failures
  uint8_t _skipSweeps[UWB_MAX_ANCHORS];   // sweeps still to skip
  uint8_t _backoffMult[UWB_MAX_ANCHORS];  // backoff multiplier: 1→2→4→8 (capped)
};

#endif // UWBRTLS_UWBSCHEDULER_H
