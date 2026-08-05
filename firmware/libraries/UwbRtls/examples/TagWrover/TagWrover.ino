/*
 * TagWrover.ino  –  UWB tag for a generic ESP32-WROVER + DW1000 module
 *                   with external SSD1306 OLED and BNO085 IMU.
 *
 * DUAL-TAG OPERATION (TagLink): two tags are rigidly mounted ~50 cm apart and
 * coordinated over a wired UART, replacing the retired over-air token ring.
 * The MASTER owns the schedule and alone publishes to the host over UDP; the
 * SLAVE sweeps only when granted its slot and returns binary results over the
 * wire; each tag's radio is TRXOFF during the other's slot. See
 * docs/dualtag_uart_link.md for the protocol, wiring and test procedure.
 *
 * Peripheral map
 * ──────────────────────────────────────────────────────────────────────
 *  DW1000 (SPI) : SCK=18  MISO=19  MOSI=23  CS=4   RST=27  IRQ=34
 *  SSD1306 OLED : SDA=32  SCL=33   (Wire,  I2C address 0x3C or 0x3D)
 *  BNO085 IMU   : SDA=25  SCL=26   (Wire1, I2C address 0x4A  ← BNO_ADDR)
 *  TagLink UART : TX=22 → peer RX=35, RX=35 ← peer TX=22, plus common GND
 *
 *  GPIO 4 = DW1000 CS on this board, so the default OLED SDA=4 conflicts.
 *  OLED moved to 32/33 (Wire). BNO085 on separate bus 25/26 (Wire1).
 *
 * ⚠  GPIO 16/17 are reserved for PSRAM on WROVER modules — do not use them.
 *    GPIO 13/14 are the charge-mode jumper. GPIO 35 is input-only (RX ok).
 *
 * Required libraries (install via Arduino Library Manager):
 *   – Adafruit BNO08x          (Adafruit)
 *   – Adafruit SSD1306          (Adafruit)
 *   – Adafruit GFX Library      (Adafruit)
 *
 * The IMU tail (,IMU,status,qw,qx,qy,qz,ax,ay,az,gx,gy,gz) is appended to the
 * master's RTLS packet when the BNO085 is present. Every published line also
 * carries a ,CYC,<cycle> tail; forwarded slave lines add ,MRX,<us> (master
 * receive time) so the host has three time references per measurement.
 */

// >>>>>>>>>>>>>>>>> PER-BOARD CONFIGURATION <<<<<<<<<<<<<<<<<<
// Role — define EXACTLY ONE. The 0xF0 board (BNO085 wired) is the master.
//   TAG_LINK_MASTER : sweeps first each cycle, grants the slave its slot,
//                     publishes both tags' lines + DIAG over UDP.
//   TAG_LINK_SLAVE  : no WiFi at all; sweeps on GO, returns data over UART.
//   TAG_LINK_SOLO   : classic single-tag free-run (no TagLink UART claimed).
#define TAG_LINK_MASTER
// #define TAG_LINK_SLAVE
// #define TAG_LINK_SOLO

// TAG_ID / ANTENNA_DELAY per board: see the block right after the #include.
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// ── Transport: master/solo publish UDP; the slave build has NO WiFi ─────────
#if defined(TAG_LINK_SLAVE)
#define UWB_HOSTLINK_SERIAL
#else
#define UWB_HOSTLINK_UDP
#endif
#define UWB_USE_OLED

// ── Board pin overrides — MUST appear before #include <UwbRtls.h> ────────────
// ESP32 UWB (non-Pro) uses DW1000 CS=4, not 21.
// GPIO 4 is also the default OLED SDA in UwbConfig.h — conflict!
// Fix: keep DW1000 CS=4 (hardware-wired) and move OLED to GPIO 32/33.
#define UWB_PIN_SS    4    // DW1000 chip select (hardware-wired on this board)
#define OLED_PIN_SDA 32    // OLED I2C SDA — moved off GPIO 4 to avoid CS clash
#define OLED_PIN_SCL 33    // OLED I2C SCL

#include <UwbRtls.h>
#include <Wire.h>
#include <esp_timer.h>
#if !defined(TAG_LINK_SLAVE)
#include <Adafruit_BNO08x.h>
#endif

// >>>>>>>>>>> PER-BOARD CONFIGURATION (continued) <<<<<<<<<<<<
// Each tag board must have a unique TAG_ID within 0xF0–0xFE, and its OWN
// calibrated ANTENNA_DELAY (re-calibrate after any radio change!).
//   Board 1 (master, has IMU): TAG_ID 0xF0
//   Board 2 (slave):           TAG_ID 0xF1, delay 16473 (3.0 m ref, 2026-06-18)
static const uint8_t  TAG_ID        = 0xF0;
static const uint16_t ANTENNA_DELAY = UWB_DEFAULT_ANTENNA_DELAY;  // calibrate!
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

static const uint8_t ANCHORS[]  = { 0x01, 0x02, 0x03, 0x04 };
static const uint8_t N_ANCHORS  = sizeof(ANCHORS) / sizeof(ANCHORS[0]);

#if defined(UWB_HOSTLINK_UDP)
static const char*    WIFI_SSID = "iitk";          // ← fill in
static const char*    WIFI_PASS = "";                    // ← "" for open network
static IPAddress      HOST_IP(255, 255, 255, 255);       // ← Python/MATLAB PC IP
static const uint16_t HOST_PORT  = 4100;
#endif

// BNO085 I2C bus  ── see PSRAM warning above ──────────────────────────────────
#define BNO_SDA   25   // Wire1 SDA — free from OLED and DW1000
#define BNO_SCL   26   // Wire1 SCL
#define BNO_ADDR  0x4A // SA0 LOW (default); use 0x4B if SA0 is tied HIGH

// Survey averaging samples (100 per anchor pair)
static const uint16_t SURVEY_SAMPLES = 100;

// Hardware antenna-delay calibration (GUI-triggered binary search per anchor).
static const uint8_t  CALIB_ITERS    = 14;
static const uint16_t CALIB_SAMPLES  = 50;
static const uint16_t CALIB_DELAY_LO = 15800;
static const uint16_t CALIB_DELAY_HI = 16900;
static const uint16_t CALIB_VERIFY_SAMPLES = 30;  // residual check at the committed delay

// Calibration owns the radio exclusively: while true, loop() freezes the TDMA
// cycle (no sweep/grant) so nothing wakes the radio mid-calibration.
static bool g_calibrating = false;

// ── Objects ───────────────────────────────────────────────────────────────────
TwrEngine    engine;
UwbScheduler scheduler;
TagLink      tagLink;    // wired dual-tag TDMA (master/slave over UART2)
HostLink     host;
OledStatus   oled;

#if defined(TAG_LINK_MASTER)
static const TagRole TAG_LINK_ROLE = TAG_ROLE_MASTER;
#elif defined(TAG_LINK_SLAVE)
static const TagRole TAG_LINK_ROLE = TAG_ROLE_SLAVE;
#else
static const TagRole TAG_LINK_ROLE = TAG_ROLE_SOLO;
#endif

// ── IMU (compiled out on the slave build — that board has no BNO085) ─────────
#if !defined(TAG_LINK_SLAVE)

Adafruit_BNO08x bno085(-1);    // -1 = MCU does not drive the reset pin
static bool     imuPresent = false;

// Latest IMU readings — updated by pollImu()
static struct {
  float   qw=1, qx=0, qy=0, qz=0;   // rotation vector (unit quaternion)
  float   ax=0, ay=0, az=0;          // linear acceleration, m/s² (gravity removed)
  float   gx=0, gy=0, gz=0;          // calibrated angular velocity, rad/s
  float   roll=0, pitch=0, yaw=0;    // Euler angles, degrees
  uint8_t status = 0;                 // BNO085 accuracy 0–3 (from val.status)
  bool    valid  = false;
} imuData;

static void quatToRPY(float qw, float qx, float qy, float qz,
                      float& roll, float& pitch, float& yaw) {
  // Roll (rotation about X axis)
  float sinr = 2.0f * (qw*qx + qy*qz);
  float cosr = 1.0f - 2.0f * (qx*qx + qy*qy);
  roll = atan2f(sinr, cosr) * 180.0f / (float)PI;

  // Pitch (rotation about Y axis)
  float sinp = 2.0f * (qw*qy - qz*qx);
  pitch = (fabsf(sinp) >= 1.0f) ? copysignf(90.0f, sinp)
                                 : asinf(sinp) * 180.0f / (float)PI;

  // Yaw (rotation about Z axis)
  float siny = 2.0f * (qw*qz + qx*qy);
  float cosy = 1.0f - 2.0f * (qy*qy + qz*qz);
  yaw = atan2f(siny, cosy) * 180.0f / (float)PI;
}

static void setImuReports() {
  // 10 000 µs interval = 100 Hz output rate for all reports.
  bno085.enableReport(SH2_ROTATION_VECTOR,       10000);
  bno085.enableReport(SH2_LINEAR_ACCELERATION,   10000);
  bno085.enableReport(SH2_GYROSCOPE_CALIBRATED,  10000);
}

// Drain the BNO085 FIFO.  Safe to call at any time; no-op if IMU absent.
// This is the master's idle work while the slave sweeps (TagLink waitDone).
static void pollImu() {
  if (!imuPresent) return;

  // If the sensor rebooted (e.g. brown-out), re-enable reports.
  if (bno085.wasReset()) {
    Serial.println("[IMU] BNO085 reset detected — re-enabling reports");
    setImuReports();
  }

  sh2_SensorValue_t val;
  while (bno085.getSensorEvent(&val)) {
    switch (val.sensorId) {
      case SH2_ROTATION_VECTOR:
        imuData.qw     = val.un.rotationVector.real;
        imuData.qx     = val.un.rotationVector.i;
        imuData.qy     = val.un.rotationVector.j;
        imuData.qz     = val.un.rotationVector.k;
        imuData.status = val.status;   // 0–3 fusion accuracy from BNO085
        quatToRPY(imuData.qw, imuData.qx, imuData.qy, imuData.qz,
                  imuData.roll, imuData.pitch, imuData.yaw);
        imuData.valid  = true;
        break;
      case SH2_LINEAR_ACCELERATION:
        imuData.ax = val.un.linearAcceleration.x;
        imuData.ay = val.un.linearAcceleration.y;
        imuData.az = val.un.linearAcceleration.z;
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        imuData.gx = val.un.gyroscope.x;
        imuData.gy = val.un.gyroscope.y;
        imuData.gz = val.un.gyroscope.z;
        break;
      default:
        break;
    }
  }
}

// Throttled Serial print (every 200 ms)
static void printImuSerial() {
  static uint32_t lastMs = 0;
  uint32_t now = millis();
  if (now - lastMs < 200) return;
  lastMs = now;

  if (!imuPresent) {
    Serial.println("[IMU] not present");
    return;
  }
  if (!imuData.valid) {
    Serial.println("[IMU] waiting for first sample...");
    return;
  }
  Serial.printf("[IMU] R:%+7.2f  P:%+7.2f  Y:%+7.2f deg    "
                "Ax:%+7.3f  Ay:%+7.3f  Az:%+7.3f m/s2    "
                "Gx:%+6.3f  Gy:%+6.3f  Gz:%+6.3f rad/s  conf=%u\n",
                imuData.roll, imuData.pitch, imuData.yaw,
                imuData.ax,   imuData.ay,   imuData.az,
                imuData.gx,   imuData.gy,   imuData.gz,
                (unsigned)imuData.status);
}

#else  // TAG_LINK_SLAVE: no IMU hardware on this board
static const bool imuPresent = false;
static void pollImu() {}
static void printImuSerial() {}
#endif // !TAG_LINK_SLAVE

// Update OLED: UWB + link status on rows 0-1, IMU data on rows 2-3.
static void updateOled(uint8_t nGood, float rateHz) {
  char l0[22], l1[22], l2[22], l3[22];
  snprintf(l0, 22, "TAG:%02X  %u/%u OK", TAG_ID, nGood, N_ANCHORS);
  if (TAG_LINK_ROLE == TAG_ROLE_MASTER)
    snprintf(l1, 22, "M %4.1fHz %s c%u", rateHz,
             tagLink.slavePresent() ? "S:ok" : "S:--", tagLink.stats().crcErr);
  else if (TAG_LINK_ROLE == TAG_ROLE_SLAVE)
    snprintf(l1, 22, "S %s cyc%u c%u", tagLink.masterSeen() ? "M:ok" : "M:--",
             tagLink.cycleId(), tagLink.stats().crcErr);
  else
    snprintf(l1, 22, "SOLO %4.1fHz", rateHz);
#if !defined(TAG_LINK_SLAVE)
  if (!imuPresent) {
    snprintf(l2, 22, "IMU not found");
    l3[0] = '\0';
  } else if (!imuData.valid) {
    snprintf(l2, 22, "IMU init...");
    l3[0] = '\0';
  } else {
    snprintf(l2, 22, "R%+5.1f P%+5.1f Y%+5.1f",
             imuData.roll, imuData.pitch, imuData.yaw);
    snprintf(l3, 22, "a:%+5.2f%+5.2f%+5.2f",
             imuData.ax, imuData.ay, imuData.az);
  }
#else
  snprintf(l2, 22, "sweeps on GO only");
  l3[0] = '\0';
#endif
  oled.show(l0, l1, l2, l3);
}

// ── Anchor self-survey (bench utility; run with the other tag POWERED OFF) ───
static void runSurvey() {
  uint8_t pairs = (N_ANCHORS * (N_ANCHORS - 1)) / 2;
  char buf[64];

  engine.setRadioState(RADIO_ACTIVE);
  snprintf(buf, sizeof(buf), "SURVEY_BEGIN,v1,%u\n", pairs);
  host.sendRaw(buf);
  Serial.printf("Survey: %u anchors, %u pairs, %u samples each\n",
                N_ANCHORS, pairs, SURVEY_SAMPLES);
  char oledStatus[20];
  snprintf(oledStatus, sizeof(oledStatus), "%u pairs, %u samp", pairs, SURVEY_SAMPLES);
  oled.showSplash("SURVEY", "Running...", oledStatus);

  uint8_t pairIdx = 0;
  for (uint8_t i = 0; i < N_ANCHORS; i++) {
    for (uint8_t j = i + 1; j < N_ANCHORS; j++) {
      uint8_t a = ANCHORS[i], b = ANCHORS[j];
      float   sum = 0.0f;
      uint16_t ok = 0;
      pairIdx++;

      char oledL1[20], oledL2[20];
      snprintf(oledL1, sizeof(oledL1), "%u/%u  %02X->%02X", pairIdx, pairs, a, b);
      Serial.printf("  Pair %u/%u  0x%02X -> 0x%02X:\n", pairIdx, pairs, a, b);

      for (uint16_t s = 0; s < SURVEY_SAMPLES; s++) {
        float dist, rxp;
        if (engine.surveyRequest(a, b, dist, rxp)) { sum += dist; ok++; }
        if ((s & 0xF) == 0xF) {
          snprintf(oledL2, sizeof(oledL2), "%u/%u ok=%u", s+1, SURVEY_SAMPLES, ok);
          oled.showSplash("SURVEY", oledL1, oledL2);
          Serial.printf("    [%u/%u ok=%u]\n", s + 1, SURVEY_SAMPLES, ok);
        }
        delay(5);
      }

      if (ok >= SURVEY_SAMPLES / 4) {
        uint32_t mm = (uint32_t)lroundf(sum / ok * 1000.0f);
        snprintf(buf, sizeof(buf), "SURVEY,v1,%u,%u,%lu,%u\n", a, b, mm, ok);
        host.sendRaw(buf);
        Serial.printf("  => %.3f m  (%u/%u ok)\n", sum / ok, ok, SURVEY_SAMPLES);
      } else {
        Serial.printf("  FAILED (ok=%u/%u) — check link 0x%02X<->0x%02X\n",
                      ok, SURVEY_SAMPLES, a, b);
      }
    }
  }
  host.sendRaw("SURVEY_DONE,v1\n");
  Serial.println("Survey complete.");
  oled.showSplash("SURVEY", "DONE", "See host PC");
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(300);

  // ── OLED first — must precede WiFi ─────────────────────────────────────────
  oled.begin();
  oled.showSplash("TAG WROVER", "Starting...", "");

  ChargeMode::begin();
  ChargeMode::checkAndSleep(&oled);   // short GPIO13-14 -> charge mode (deep sleep)

#if !defined(TAG_LINK_SLAVE)
  // ── BNO085 on Wire1 (pins BNO_SDA, BNO_SCL) ──────────────────────────────
  Wire1.begin(BNO_SDA, BNO_SCL);
  Serial.printf("[IMU] probing BNO085 at 0x%02X on SDA=%d SCL=%d ...\n",
                BNO_ADDR, BNO_SDA, BNO_SCL);

  if (bno085.begin_I2C(BNO_ADDR, &Wire1)) {
    imuPresent = true;
    setImuReports();
    Serial.println("[IMU] BNO085 found and configured");
    oled.showSplash("TAG WROVER", "BNO085 OK", "WiFi...");
  } else {
    imuPresent = false;
    Serial.println("[IMU] BNO085 NOT FOUND — check wiring and BNO_SDA/SCL above.");
    oled.showSplash("TAG WROVER", "IMU MISSING", "Check pins");
    delay(2000);
  }
#endif

  // ── WiFi / transport init ────────────────────────────────────────────────
#if defined(UWB_HOSTLINK_UDP)
  host.begin(WIFI_SSID, WIFI_PASS, HOST_IP, HOST_PORT);
  // WiFi association can glitch I2C buses; re-init both afterwards.
  Wire.end();
  delay(50);
  oled.begin();
#if !defined(TAG_LINK_SLAVE)
  if (imuPresent) {
    Wire1.end();
    delay(20);
    Wire1.begin(BNO_SDA, BNO_SCL);
    setImuReports();   // re-enable reports; bus restart may have lost the session
  }
#endif
#else
  host.begin(115200);
#endif

  // ── UWB radio LAST — after WiFi/I2C settled ────────────────────────────────
  // Explicit SPI.begin() here because WiFi + Wire bus teardowns above can leave
  // the ESP32 SPI peripheral in an undefined state on WROVER modules.
  // "DECA 01302001" in Serial → CS=4 correct; all zeros → wrong CS pin.
  SPI.begin(UWB_PIN_SCK, UWB_PIN_MISO, UWB_PIN_MOSI);
  engine.begin(TWR_TAG, TAG_ID, ANTENNA_DELAY);
  scheduler.begin(&engine, ANCHORS, N_ANCHORS);
  engine.printDeviceId();

  // ── TagLink UART (inert for SOLO). Slave parks its radio until granted. ───
  tagLink.begin(TAG_LINK_ROLE, TAG_ID, imuPresent ? TAGLINK_CAP_IMU : 0);
  if (TAG_LINK_ROLE == TAG_ROLE_SLAVE)
    engine.setRadioState(RADIO_IDLE);

  Serial.printf("Tag 0x%02X ready — role %s, CS=%d, %u anchors, IMU:%s\n",
                TAG_ID,
                TAG_LINK_ROLE == TAG_ROLE_MASTER ? "MASTER" :
                TAG_LINK_ROLE == TAG_ROLE_SLAVE  ? "SLAVE"  : "SOLO",
                UWB_PIN_SS, N_ANCHORS, imuPresent ? "YES" : "NO");
  char l2[20];
  snprintf(l2, 20, "%u anch  IMU:%s", N_ANCHORS, imuPresent ? "OK" : "NO");
  oled.showSplash("TAG WROVER",
                  TAG_LINK_ROLE == TAG_ROLE_MASTER ? "MASTER" :
                  TAG_LINK_ROLE == TAG_ROLE_SLAVE  ? "SLAVE"  : "SOLO",
                  l2, "Ready");
}

// ── Antenna delay push relay ──────────────────────────────────────────────────
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
  snprintf(buf, sizeof(buf), ok ? "ANTDELAY_ACK,%u,%u\n" : "ANTDELAY_FAIL,%u,%u\n",
           anchorAddr, delayTicks);
  host.sendRaw(buf);
  Serial.printf("[CAL] %s 0x%02X\n", ok ? "ACK from" : "No ACK from", anchorAddr);
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
    if (!engine.pushAntDelay(anchorAddr, mid, 3)) {   // retries 1->3: survive a transient miss
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

// Shared command polling (UDP for master, USB serial for both — bench use).
static void pollCommands() {
  static char udpCmd[32];
  if (host.receiveCmd(udpCmd, sizeof(udpCmd)) > 0)
    dispatchCmd(udpCmd);

  static char    cmdBuf[32];
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

// ── MASTER / SOLO cycle ───────────────────────────────────────────────────────
// The slave's slot is NOT wait time: GO is issued the moment our own sweep
// ends, and everything below "grantSlot" (IMU, format, UDP, DIAG, OLED) runs
// while the slave is sweeping. waitDone() only blocks for whatever budget is
// left, polling the IMU as idle work.
#if !defined(TAG_LINK_SLAVE)
static void masterCycle() {
  static float rateHz = 0.0f;

  int64_t  cycleStartUs = esp_timer_get_time();
  uint16_t cycle = tagLink.startCycle();

  // 1) Our own sweep, then free the air.
  int64_t t0 = esp_timer_get_time();
  uint8_t nGood = scheduler.sweep();
  uint32_t ownSweepUs = (uint32_t)(esp_timer_get_time() - t0);
  engine.setRadioState(RADIO_IDLE);

  // 2) Grant the slave its slot FIRST — its sweep now overlaps our work.
  bool granted = false;
  if (TAG_LINK_ROLE == TAG_ROLE_MASTER && tagLink.slavePresent()) {
    tagLink.grantSlot();
    granted = true;
  }

  // 3) Master's own processing, concurrent with the slave's sweep.
  int64_t procT0 = esp_timer_get_time();
  pollImu();
  ImuSample imuSamp;
#if !defined(TAG_LINK_SLAVE)
  if (imuPresent && imuData.valid) {
    imuSamp.valid  = true;
    imuSamp.status = imuData.status;
    imuSamp.qw = imuData.qw;  imuSamp.qx = imuData.qx;
    imuSamp.qy = imuData.qy;  imuSamp.qz = imuData.qz;
    imuSamp.ax = imuData.ax;  imuSamp.ay = imuData.ay;  imuSamp.az = imuData.az;
    imuSamp.gx = imuData.gx;  imuSamp.gy = imuData.gy;  imuSamp.gz = imuData.gz;
  }
#endif

  char line[768];
  int len = HostLink::format(line, sizeof(line), millis(), TAG_ID, scheduler,
                             imuSamp.valid ? &imuSamp : nullptr);
  if (len > 0) {
    len = HostLink::appendTail(line, sizeof(line), len, ",CYC,%u", cycle);
    host.sendLine(line, len);
  }

  printImuSerial();
  static uint32_t oledLast = 0;
  if (millis() - oledLast > 500) {
    oledLast = millis();
    updateOled(nGood, rateHz);
  }
  uint32_t procUs = (uint32_t)(esp_timer_get_time() - procT0);

  // 4) Collect the slave's slot (IMU keeps draining as idle work).
  uint32_t slaveSweepUs = 0;
  if (granted) {
    tagLink.waitDone(pollImu);
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

  // 5) Per-cycle time budget — the Phase 2 "where does the time go" input.
  //    DIAG,v1,<cycle>,<cycle_start_us>,<master_sweep_us>,<slave_sweep_us>,
  //            <uart_wait_us>,<proc_us>,<idle_us>,<total_us>
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

  // Rate-limited human-readable summary (+ link health: crc spike = bad cable).
  static uint32_t timeLast = 0;
  if (millis() - timeLast > 2000) {
    timeLast = millis();
    const TagLinkStats& s = tagLink.stats();
    Serial.printf("[TIME] cycle=%u own=%.1fms slave=%.1fms uart=%.1fms proc=%.1fms"
                  " total=%.1fms rate=%.1fHz | crc=%lu resync=%lu stale=%lu"
                  " gap=%lu dup=%lu tmo=%lu\n",
                  cycle, ownSweepUs / 1e3f, slaveSweepUs / 1e3f, uartWaitUs / 1e3f,
                  procUs / 1e3f, totalUs / 1e3f, rateHz,
                  (unsigned long)s.crcErr, (unsigned long)s.resync,
                  (unsigned long)s.staleCycle, (unsigned long)s.seqGap,
                  (unsigned long)s.seqDup, (unsigned long)s.slotTimeout);
  }
}
#endif // !TAG_LINK_SLAVE

// ── SLAVE cycle ───────────────────────────────────────────────────────────────
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
    updateOled(nGood, 0.0f);
  }
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop() {
  ChargeMode::checkAndSleep(&oled);   // jumper 13-14 enters charge mode at any time
  pollCommands();

  // Calibration owns the radio exclusively: freeze the TDMA cycle while it runs
  // so no sweep/grant/slave-GO can wake the radio mid-calibration.
  if (g_calibrating) return;

  tagLink.poll();
  if (TAG_LINK_ROLE == TAG_ROLE_SLAVE) {
    slaveCycle();
  } else {
#if !defined(TAG_LINK_SLAVE)
    masterCycle();   // MASTER and SOLO (SOLO = master with no slave present)
#endif
  }
}
