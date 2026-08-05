/*
 * Tag.ino - UWB RTLS tag (initiator) for the Makerfabs "ESP32 UWB Pro with
 *           Display" board (DW1000 CS=21, OLED on 4/5).
 *
 * The tag ranges to every anchor in ANCHORS[] each sweep and streams the raw
 * distances to the host. It NEVER solves position itself, so adding anchors
 * only means editing ANCHORS[] here and the matching host config - no
 * algorithm change, and no firmware change once anchors are added.
 *
 * DUAL-TAG OPERATION (TagLink): multi-tag coordination runs over a wired UART
 * (TX=22 -> peer RX=35, RX=35 <- peer TX=22, common GND), replacing the
 * retired over-air token ring. Pick a role below; SOLO is the classic
 * single-tag free-run. See docs/dualtag_uart_link.md.
 *
 * Transport: the master/solo roles pick ONE of the two #defines below; the
 * slave build forces serial (it must not run WiFi at all).
 *   UWB_HOSTLINK_SERIAL : stream over USB serial (simplest; tethered)
 *   UWB_HOSTLINK_UDP    : stream over WiFi UDP to the host PC
 */

// >>>>>>>>>>>>>>>>> PER-BOARD CONFIGURATION <<<<<<<<<<<<<<<<<<
// Role — define EXACTLY ONE (see TagWrover.ino for the dual-tag rig wiring).
// #define TAG_LINK_MASTER
// #define TAG_LINK_SLAVE
#define TAG_LINK_SOLO

// Each tag board must have a unique TAG_ID within 0xF0–0xFE.
// ANTENNA_DELAY must be re-calibrated per board after any radio change.
//   Tag 1 (0xF0): 16360   Tag 2 (0xF1): 16473
static const uint8_t  TAG_ID        = 0xF1;  // change per board
static const uint16_t ANTENNA_DELAY = 16473;  // Tag 2 calibrated (3.0 m ref, 2026-06-18)
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// ---- transport (compile-time; slave forces serial) ----
#if defined(TAG_LINK_SLAVE)
#define UWB_HOSTLINK_SERIAL
#else
// #define UWB_HOSTLINK_SERIAL
#define UWB_HOSTLINK_UDP
#endif
#define UWB_USE_OLED

#include <UwbRtls.h>
#include <esp_timer.h>

// Anchor short addresses. ADD ANCHORS HERE (mirror in the host config).
static const uint8_t ANCHORS[]  = { 0x01, 0x02, 0x03, 0x04 };
static const uint8_t N_ANCHORS  = sizeof(ANCHORS) / sizeof(ANCHORS[0]);

#if defined(UWB_HOSTLINK_UDP)
static const char*    WIFI_SSID  = "your-wifi-ssid";   // ← per deployment
static const char*    WIFI_PASS  = "";                     // "" for an open network
static IPAddress      HOST_IP(255, 255, 255, 255);    // Python/MATLAB PC — update per machine
static const uint16_t HOST_PORT  = 4100;
#endif

// Number of ranging samples averaged per anchor pair during self-survey.
// 100 samples ≈ 4 s per pair; 6 pairs (4 anchors) ≈ 24 s total.
static const uint16_t SURVEY_SAMPLES = 100;

// Hardware antenna-delay calibration (GUI-triggered binary search per anchor).
static const uint8_t  CALIB_ITERS    = 14;    // iterations to converge (~0.07 tick resolution)
static const uint16_t CALIB_SAMPLES  = 50;    // samples per iteration (~250 ms ranging)
static const uint16_t CALIB_DELAY_LO = 15800; // binary-search lower bound (ticks)
static const uint16_t CALIB_DELAY_HI = 16900; // binary-search upper bound (ticks)
static const uint16_t CALIB_VERIFY_SAMPLES = 30;  // residual check at the committed delay

// Calibration owns the radio exclusively: while true, loop() freezes the TDMA
// cycle (no sweep/grant) so nothing wakes the radio mid-calibration.
static bool g_calibrating = false;

TwrEngine    engine;
UwbScheduler scheduler;
TagLink      tagLink;    // wired dual-tag TDMA (inert in SOLO)
HostLink     host;
OledStatus   oled;
SensorImu    imu;     // stub today; populates the IMU,... packet tail later

#if defined(TAG_LINK_MASTER)
static const TagRole TAG_LINK_ROLE = TAG_ROLE_MASTER;
#elif defined(TAG_LINK_SLAVE)
static const TagRole TAG_LINK_ROLE = TAG_ROLE_SLAVE;
#else
static const TagRole TAG_LINK_ROLE = TAG_ROLE_SOLO;
#endif

static void splashIdent() {
  char title[10], l1[20], l2[20];
  snprintf(title, sizeof(title), "TAG  %02X", TAG_ID);
  snprintf(l1,    sizeof(l1),    "Delay: %u", ANTENNA_DELAY);
  snprintf(l2,    sizeof(l2),    "%u anchors", N_ANCHORS);
  oled.showSplash(title, l1, l2,
                  TAG_LINK_ROLE == TAG_ROLE_MASTER ? "MASTER" :
                  TAG_LINK_ROLE == TAG_ROLE_SLAVE  ? "SLAVE"  : "SOLO");
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // Init OLED before WiFi — WiFi startup corrupts I2C if OLED init runs after.
  oled.begin();
  { char t[10]; snprintf(t, sizeof(t), "TAG  %02X", TAG_ID); oled.showSplash(t, "Starting...", ""); }

  ChargeMode::begin();
  ChargeMode::checkAndSleep(&oled);   // short GPIO13-14 -> charge mode (deep sleep)

#if defined(UWB_HOSTLINK_UDP)
  host.begin(WIFI_SSID, WIFI_PASS, HOST_IP, HOST_PORT);
  // Re-init I2C after WiFi — WiFi RF can glitch the bus during association.
  Wire.end();
  delay(50);
  oled.begin();
#else
  host.begin(115200);
#endif
  splashIdent();

  engine.begin(TWR_TAG, TAG_ID, ANTENNA_DELAY);
  scheduler.begin(&engine, ANCHORS, N_ANCHORS);
  imu.begin();
  engine.printDeviceId();

  tagLink.begin(TAG_LINK_ROLE, TAG_ID, imu.present() ? TAGLINK_CAP_IMU : 0);
  if (TAG_LINK_ROLE == TAG_ROLE_SLAVE)
    engine.setRadioState(RADIO_IDLE);   // radio parked until the master grants

  Serial.printf("Tag 0x%02X ready, role %s, %u anchors\n", TAG_ID,
                TAG_LINK_ROLE == TAG_ROLE_MASTER ? "MASTER" :
                TAG_LINK_ROLE == TAG_ROLE_SLAVE  ? "SLAVE"  : "SOLO",
                N_ANCHORS);
}

// ---------------------------------------------------------------------------
// Anchor self-survey: ask every anchor pair (A,B) to range to each other.
// Outputs SURVEY_BEGIN / SURVEY,v1,... / SURVEY_DONE lines on HostLink.
// Triggered by sending "SURVEY\n" over Serial. Bench utility — run it with
// the other tag POWERED OFF (it transmits outside any TagLink slot).
// Wire format: SURVEY,v1,<src_id>,<dst_id>,<avg_dist_mm>,<ok_samples>
// ---------------------------------------------------------------------------
static void runSurvey() {
  uint8_t pairs = (N_ANCHORS * (N_ANCHORS - 1)) / 2;
  char buf[64];

  engine.setRadioState(RADIO_ACTIVE);
  snprintf(buf, sizeof(buf), "SURVEY_BEGIN,v1,%u\n", pairs);
  host.sendRaw(buf);
  Serial.printf("Survey: %u anchors, %u pairs, %u samples each\n",
                N_ANCHORS, pairs, SURVEY_SAMPLES);
  oled.showSplash("SURVEY", "Running...", buf);

  uint8_t pairIdx = 0;
  for (uint8_t i = 0; i < N_ANCHORS; i++) {
    for (uint8_t j = i + 1; j < N_ANCHORS; j++) {
      uint8_t a = ANCHORS[i], b = ANCHORS[j];
      float   sum = 0.0f;
      uint16_t ok = 0;
      pairIdx++;

      Serial.printf("  Pair %u/%u  0x%02X -> 0x%02X:\n", pairIdx, pairs, a, b);

      char oledL1[20], oledL2[20];
      snprintf(oledL1, sizeof(oledL1), "%u/%u  %02X->%02X", pairIdx, pairs, a, b);

      for (uint16_t s = 0; s < SURVEY_SAMPLES; s++) {
        float dist, rxp;
        if (engine.surveyRequest(a, b, dist, rxp)) {
          sum += dist;
          ok++;
        }
        if ((s & 0xF) == 0xF) {
          Serial.printf("    [%u/%u ok=%u]\n", s + 1, SURVEY_SAMPLES, ok);
          snprintf(oledL2, sizeof(oledL2), "%u/%u ok=%u", s+1, SURVEY_SAMPLES, ok);
          oled.showSplash("SURVEY", oledL1, oledL2);
        }
        delay(5);
      }

      if (ok >= SURVEY_SAMPLES / 4) {
        float mean = sum / ok;
        uint32_t mm = (uint32_t)lroundf(mean * 1000.0f);
        snprintf(buf, sizeof(buf), "SURVEY,v1,%u,%u,%lu,%u\n", a, b, mm, ok);
        host.sendRaw(buf);
        Serial.printf("  => %.3f m  (%u/%u ok)\n", mean, ok, SURVEY_SAMPLES);
      } else {
        Serial.printf("  FAILED (ok=%u/%u) — check link between 0x%02X and 0x%02X\n",
                      ok, SURVEY_SAMPLES, a, b);
      }
    }
  }

  host.sendRaw("SURVEY_DONE,v1\n");
  Serial.println("Survey complete.");
  oled.showSplash("SURVEY", "DONE", "See Serial/MATLAB");
}

// ---------------------------------------------------------------------------
// Antenna delay push: host sends "SETANTDELAY,<anchor_id>,<delay_ticks>".
// Tag relays to the anchor via UWB; reports result back to host via HostLink.
// ---------------------------------------------------------------------------
static void handleSetAntDelay(const char* cmd) {
  unsigned int aid = 0, ticks = 0;
  if (sscanf(cmd, "SETANTDELAY,%u,%u", &aid, &ticks) != 2) {
    Serial.println("[CAL] SETANTDELAY parse error — expected SETANTDELAY,<id>,<ticks>");
    return;
  }
  uint8_t  anchorAddr = (uint8_t)aid;
  uint16_t delayTicks = (uint16_t)ticks;
  Serial.printf("[CAL] Pushing delay %u to anchor 0x%02X ...\n", delayTicks, anchorAddr);
  oled.showSplash("CALIBRATE", "Push delay...", "");
  g_calibrating = true;                 // exclusive: freeze TDMA for the push
  engine.setRadioState(RADIO_ACTIVE);

  bool ok = engine.pushAntDelay(anchorAddr, delayTicks);
  char buf[48];
  if (ok) {
    snprintf(buf, sizeof(buf), "ANTDELAY_ACK,%u,%u\n", anchorAddr, delayTicks);
    Serial.printf("[CAL] ACK from 0x%02X\n", anchorAddr);
  } else {
    snprintf(buf, sizeof(buf), "ANTDELAY_FAIL,%u,%u\n", anchorAddr, delayTicks);
    Serial.printf("[CAL] No ACK from 0x%02X (anchor unreachable?)\n", anchorAddr);
  }
  host.sendRaw(buf);
  g_calibrating = false;
  engine.setRadioState(RADIO_ACTIVE);
}

// Range `n` times to anchorAddr; outputs mean distance (mm), returns #successes.
// Shared by the search (PHASE 1) and the residual verification (PHASE 2).
static uint16_t calibRangeBatch(uint8_t anchorAddr, uint16_t n, float& meanMmOut) {
  float sum = 0.0f; uint16_t ok = 0;
  for (uint16_t s = 0; s < n; s++) {
    float dist, rxp;
    if (engine.rangeTo(anchorAddr, dist, rxp)) { sum += dist; ok++; }
    delay(5);
  }
  meanMmOut = ok ? (sum / ok) * 1000.0f : 0.0f;
  return ok;
}

// ---------------------------------------------------------------------------
// Hardware antenna-delay calibration: binary-searches each ANCHOR's delay.
// Host sends "HWCALIB,<anchor_id>,<true_dist_mm>".  Tag iterates: push a
// candidate delay to the anchor, range CALIB_SAMPLES times, measure mean,
// adjust bounds.  Final value is NVS-committed on the anchor.
// Progress: HWCALIB_PROG,<id>,<iter>,<delay>,<mean_mm>,<err_mm>
// Result:   HWCALIB_DONE,<id>,<final_delay>  or  HWCALIB_FAIL,<id>,<reason>
// ---------------------------------------------------------------------------
static void handleHwCalib(const char* cmd) {
  unsigned int aid, trueDistMm;
  if (sscanf(cmd, "HWCALIB,%u,%u", &aid, &trueDistMm) != 2) {
    host.sendRaw("HWCALIB_FAIL,0,parse_error\n");
    return;
  }
  uint8_t anchorAddr = (uint8_t)aid;
  Serial.printf("[HWCAL] Calibrating anchor 0x%02X at true dist %u mm\n",
                anchorAddr, trueDistMm);
  char oledL1[20], oledL2[20];
  snprintf(oledL1, sizeof(oledL1), "Anch 0x%02X", anchorAddr);
  oled.showSplash("HW CALIB", oledL1, "Searching...");

  g_calibrating = true;                 // exclusive: calibration owns the radio
  engine.setRadioState(RADIO_ACTIVE);

  uint16_t finalDelay = 0;
  float    finalMeanMm = 0.0f;
  bool     ok_all = false;

  // ── PHASE 1 · CALIBRATE — 14-iteration binary search (logic UNCHANGED) ──
  uint16_t low = CALIB_DELAY_LO, high = CALIB_DELAY_HI;
  for (uint8_t it = 0; it < CALIB_ITERS; it++) {
    uint16_t mid = (low + high) / 2;
    if (!engine.pushAntDelay(anchorAddr, mid, 3)) {   // retries 1→3: survive a transient miss
      char b[48]; snprintf(b, sizeof(b), "HWCALIB_FAIL,%u,push_timeout\n", aid);
      host.sendRaw(b); goto done;
    }
    float meanMm;
    uint16_t ok = calibRangeBatch(anchorAddr, CALIB_SAMPLES, meanMm);
    if (ok < CALIB_SAMPLES / 4) {
      char b[48]; snprintf(b, sizeof(b), "HWCALIB_FAIL,%u,no_range\n", aid);
      host.sendRaw(b); goto done;
    }
    int32_t errMm = (int32_t)(meanMm - (float)trueDistMm);
    char b[64];
    snprintf(b, sizeof(b), "HWCALIB_PROG,%u,%u,%u,%u,%ld\n",
             aid, it, mid, (uint32_t)meanMm, (long)errMm);
    host.sendRaw(b);
    Serial.printf("  [%2u/%u] delay=%u  mean=%u mm  err=%+ld mm\n",
                  (unsigned)(it + 1), (unsigned)CALIB_ITERS,
                  mid, (uint32_t)meanMm, (long)errMm);
    if ((it & 6) == 6) {
      snprintf(oledL2, sizeof(oledL2), "it%u d=%u", it + 1, mid);
      oled.showSplash("HW CALIB", oledL1, oledL2);
    }
    if (meanMm > (float)trueDistMm) low = mid; else high = mid;
  }
  finalDelay = (low + high) / 2;

  // ── PHASE 2 · VERIFY — commit final delay, measure residual at that delay ──
  if (!engine.pushAntDelay(anchorAddr, finalDelay, 3)) {
    char b[48]; snprintf(b, sizeof(b), "HWCALIB_FAIL,%u,final_push\n", aid);
    host.sendRaw(b); goto done;
  }
  {
    float vmean;
    uint16_t ok = calibRangeBatch(anchorAddr, CALIB_VERIFY_SAMPLES, vmean);
    finalMeanMm = (ok >= CALIB_VERIFY_SAMPLES / 4) ? vmean : 0.0f;
  }
  ok_all = true;

  // ── PHASE 3 · REPORT — DONE with delay + verified mean + residual ──
  {
    int32_t residMm = (int32_t)(finalMeanMm - (float)trueDistMm);
    char b[80];
    snprintf(b, sizeof(b), "HWCALIB_DONE,%u,%u,%u,%ld\n",
             aid, finalDelay, (uint32_t)finalMeanMm, (long)residMm);
    host.sendRaw(b);
    Serial.printf("[CAL] A%02X: delay ->%u  true %u mm  final %u mm  resid %+ld mm\n",
                  anchorAddr, finalDelay, trueDistMm, (uint32_t)finalMeanMm, (long)residMm);
    snprintf(oledL2, sizeof(oledL2), "d=%u OK", finalDelay);
    oled.showSplash("HW CALIB", oledL1, oledL2);
  }

done:
  if (!ok_all) { snprintf(oledL2, sizeof(oledL2), "FAILED"); oled.showSplash("HW CALIB", oledL1, oledL2); }
  g_calibrating = false;                // release the radio; TDMA resumes next loop()
  engine.setRadioState(RADIO_ACTIVE);
}

static void dispatchCmd(const char* cmd) {
  if (strcmp(cmd, "SURVEY") == 0)
    runSurvey();
  else if (strncmp(cmd, "SETANTDELAY,", 12) == 0)
    handleSetAntDelay(cmd);
  else if (strncmp(cmd, "HWCALIB,", 8) == 0)
    handleHwCalib(cmd);
}

static void pollCommands() {
  // Commands from the host via WiFi UDP (e.g. SETANTDELAY from calibration tool).
  static char udpCmd[32];
  if (host.receiveCmd(udpCmd, sizeof(udpCmd)) > 0)
    dispatchCmd(udpCmd);

  // Non-blocking check for serial commands.
  static char cmdBuf[32];
  static uint8_t cmdLen = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      cmdBuf[cmdLen] = '\0';
      dispatchCmd(cmdBuf);
      cmdLen = 0;
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
}

// MASTER / SOLO cycle: sweep, free the air, grant the slave FIRST, then do our
// own publishing while the slave sweeps; collect + forward its data; emit DIAG.
static void masterCycle() {
  static float rateHz = 0.0f;
  static uint8_t nGood = 0;

  int64_t  cycleStartUs = esp_timer_get_time();
  uint16_t cycle = tagLink.startCycle();

  int64_t t0 = esp_timer_get_time();
  nGood = scheduler.sweep();
  uint32_t ownSweepUs = (uint32_t)(esp_timer_get_time() - t0);
  engine.setRadioState(RADIO_IDLE);

  bool granted = false;
  if (TAG_LINK_ROLE == TAG_ROLE_MASTER && tagLink.slavePresent()) {
    tagLink.grantSlot();
    granted = true;
  }

  // Our processing overlaps the slave's sweep.
  int64_t procT0 = esp_timer_get_time();
  ImuSample s;
  bool hasImu = imu.read(s);
  char line[768];
  int len = HostLink::format(line, sizeof(line), millis(), TAG_ID, scheduler,
                             hasImu ? &s : nullptr);
  if (len > 0) {
    len = HostLink::appendTail(line, sizeof(line), len, ",CYC,%u", cycle);
    host.sendLine(line, len);
  }

  static uint32_t oledLast = 0;
  if (millis() - oledLast > 500) {
    oledLast = millis();
    char title[10], l1[20], l2[20];
    snprintf(title, sizeof(title), "TAG  %02X", TAG_ID);
    snprintf(l1,    sizeof(l1),    "%4.1fHz %s", rateHz,
             TAG_LINK_ROLE == TAG_ROLE_MASTER
               ? (tagLink.slavePresent() ? "S:ok" : "S:--") : "SOLO");
    snprintf(l2,    sizeof(l2),    "%u/%u anchors OK", nGood, N_ANCHORS);
    oled.showSplash(title, l1, l2);
  }
  uint32_t procUs = (uint32_t)(esp_timer_get_time() - procT0);

  // Collect the slave's slot.
  uint32_t slaveSweepUs = 0;
  if (granted) {
    tagLink.waitDone();
    if (tagLink.hasData()) {
      slaveSweepUs = tagLink.dataSweepUs();
      int slen = HostLink::format(line, sizeof(line), tagLink.dataTagMs(),
                                  tagLink.dataTagId(), tagLink.dataRanges(),
                                  tagLink.dataCount());
      if (slen > 0) {
        slen = HostLink::appendTail(line, sizeof(line), slen, ",CYC,%u,MRX,%llu",
                                    cycle, (unsigned long long)tagLink.dataRxTimeUs());
        host.sendLine(line, slen);
      }
    }
  }

  // Per-cycle time budget (see docs/dualtag_uart_link.md for field meanings).
  uint32_t uartWaitUs = granted ? tagLink.lastUartWaitUs() : 0;
  uint32_t totalUs    = (uint32_t)(esp_timer_get_time() - cycleStartUs);
  uint32_t accounted  = ownSweepUs + procUs + uartWaitUs;
  uint32_t idleUs     = (totalUs > accounted) ? (totalUs - accounted) : 0;
  char diag[128];
  int dlen = snprintf(diag, sizeof(diag),
                      "DIAG,v1,%u,%lld,%lu,%lu,%lu,%lu,%lu,%lu\n",
                      cycle, (long long)cycleStartUs,
                      (unsigned long)ownSweepUs, (unsigned long)slaveSweepUs,
                      (unsigned long)uartWaitUs, (unsigned long)procUs,
                      (unsigned long)idleUs, (unsigned long)totalUs);
  if (dlen > 0 && dlen < (int)sizeof(diag)) host.sendLine(diag, dlen);

  if (totalUs > 0) {
    float hz = 1e6f / (float)totalUs;
    rateHz = (rateHz == 0.0f) ? hz : rateHz + 0.2f * (hz - rateHz);
  }

  static uint32_t timeLast = 0;
  if (millis() - timeLast > 2000) {
    timeLast = millis();
    const TagLinkStats& st = tagLink.stats();
    Serial.printf("[TIME] cycle=%u own=%.1fms slave=%.1fms uart=%.1fms proc=%.1fms"
                  " total=%.1fms rate=%.1fHz | crc=%lu resync=%lu stale=%lu"
                  " gap=%lu dup=%lu tmo=%lu\n",
                  cycle, ownSweepUs / 1e3f, slaveSweepUs / 1e3f, uartWaitUs / 1e3f,
                  procUs / 1e3f, totalUs / 1e3f, rateHz,
                  (unsigned long)st.crcErr, (unsigned long)st.resync,
                  (unsigned long)st.staleCycle, (unsigned long)st.seqGap,
                  (unsigned long)st.seqDup, (unsigned long)st.slotTimeout);
  }
}

// SLAVE cycle: radio parked; on GO sweep and return binary results + DONE.
static void slaveCycle() {
  uint16_t cycle, budgetMs;
  if (!tagLink.pollGo(cycle, budgetMs)) return;

  engine.setRadioState(RADIO_ACTIVE);
  int64_t t0 = esp_timer_get_time();
  uint8_t nGood = scheduler.sweep();
  uint32_t sweepUs = (uint32_t)(esp_timer_get_time() - t0);
  engine.setRadioState(RADIO_IDLE);

  RangeResult results[UWB_MAX_ANCHORS];
  uint8_t n = scheduler.anchorCount();
  if (n > UWB_MAX_ANCHORS) n = UWB_MAX_ANCHORS;
  for (uint8_t i = 0; i < n; i++) results[i] = scheduler.result(i);

  tagLink.sendData(millis(), TAG_ID, sweepUs, results, n);
  tagLink.sendDone();

  // Serial mirror for bench debugging (same serializer as the master uses).
  char line[768];
  int len = HostLink::format(line, sizeof(line), millis(), TAG_ID, results, n);
  if (len > 0) Serial.write(line, len);

  static uint32_t oledLast = 0;
  if (millis() - oledLast > 500) {
    oledLast = millis();
    char title[10], l1[20], l2[20];
    snprintf(title, sizeof(title), "TAG  %02X", TAG_ID);
    snprintf(l1,    sizeof(l1),    "SLAVE %s cyc%u",
             tagLink.masterSeen() ? "M:ok" : "M:--", cycle);
    snprintf(l2,    sizeof(l2),    "%u/%u anchors OK", nGood, N_ANCHORS);
    oled.showSplash(title, l1, l2);
  }
}

void loop() {
  ChargeMode::checkAndSleep(&oled);   // jumper 13-14 enters charge mode at any time
  pollCommands();

  // Calibration owns the radio exclusively: freeze the TDMA cycle while it runs
  // so no sweep/grant/slave-GO can wake the radio mid-calibration.
  if (g_calibrating) return;

  tagLink.poll();
  if (TAG_LINK_ROLE == TAG_ROLE_SLAVE)
    slaveCycle();
  else
    masterCycle();   // MASTER and SOLO (SOLO = master with no slave present)
}
