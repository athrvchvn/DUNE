#include "UwbScheduler.h"

void UwbScheduler::begin(TwrEngine* engine, const uint8_t* anchorAddrs, uint8_t n) {
  _engine = engine;
  _n = (n > UWB_MAX_ANCHORS) ? UWB_MAX_ANCHORS : n;
  for (uint8_t i = 0; i < _n; i++) {
    _addrs[i]          = anchorAddrs[i];
    _results[i].id     = anchorAddrs[i];
    _results[i].valid  = false;
    _failStreak[i]     = 0;
    _skipSweeps[i]     = 0;
    _backoffMult[i]    = 1;
  }
}

uint8_t UwbScheduler::sweep() {
  uint8_t good = 0;
  _sweepSeq++;
  for (uint8_t i = 0; i < _n; i++) {
    _results[i].id = _addrs[i];

    // Dead-anchor backoff: skip this slot and count down.
    if (_skipSweeps[i] > 0) {
      _skipSweeps[i]--;
      _results[i].valid    = false;
      _results[i].distance = 0.0f;
      _results[i].rxPower  = 0.0f;
      _results[i].fpPower  = 0.0f;
      _results[i].quality  = 0.0f;
      continue;
    }

    float dist = 0.0f, rxp = 0.0f;
    bool ok = _engine->rangeTo(_addrs[i], dist, rxp);
    _results[i].valid    = ok;
    _results[i].distance = ok ? dist : 0.0f;
    _results[i].rxPower  = ok ? rxp  : 0.0f;
    _results[i].fpPower  = ok ? _engine->fpPower() : 0.0f;
    _results[i].quality  = ok ? _engine->quality()  : 0.0f;

    if (ok) {
      if (_failStreak[i] > 0) {
        Serial.printf("[SCHED] anchor 0x%02X recovered\n", _addrs[i]);
      }
      _failStreak[i]  = 0;
      _backoffMult[i] = 1;   // reset backoff on recovery
      good++;
    } else {
      _failStreak[i]++;
      if (_failStreak[i] >= SKIP_AFTER_FAILS) {
        uint8_t skip = SKIP_BASE_SWEEPS * _backoffMult[i];
        if (skip > SKIP_MAX_SWEEPS) skip = SKIP_MAX_SWEEPS;
        _skipSweeps[i]  = skip;
        _failStreak[i]  = 0;
        Serial.printf("[SCHED] anchor 0x%02X: %u fails -> skip %u sweeps\n",
                      _addrs[i], SKIP_AFTER_FAILS, skip);
        if (_backoffMult[i] < 8) _backoffMult[i] *= 2;
      }
    }
  }
  return good;
}
