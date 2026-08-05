/*
 * Anchor.ino - UWB RTLS anchor (responder).
 *
 * Flash this to each anchor board. The ONLY things you change per board are
 * ANCHOR_ID and ANTENNA_DELAY (factory default). At runtime the host can push
 * a calibrated delay via SETANTDELAY which is stored in NVS and overrides this
 * constant on every subsequent boot — no reflash needed.
 *
 * Board: Makerfabs ESP32 UWB Pro with Display (DW1000).
 * SPI is initialised by the driver on the default ESP32 VSPI pins (18/19/23),
 * which match this board - do not call SPI.begin() yourself.
 */
#define UWB_USE_OLED            // on-board SSD1306 display
#define UWB_HOSTLINK_SERIAL     // serial debug prints

#include <UwbRtls.h>
#include <Preferences.h>

// >>>>>>>>>>>>>>>>> SET PER BOARD <<<<<<<<<<<<<<<<<<
static const uint8_t  ANCHOR_ID     = 0x04;  // unique: 0x01, 0x02, 0x03, ...
static const uint16_t ANTENNA_DELAY = 16434; // factory default — overridden by NVS if calibrated
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

TwrEngine  engine;
OledStatus oled;
Preferences prefs;

// NVS is committed once, after the delay has been quiet for NVS_COMMIT_MS —
// not on every push. A HWCALIB binary search pushes a new delay ~15 times in
// quick succession; writing NVS on each one blocks this loop with a flash
// write right when the tag's next ranging batch needs the anchor responsive,
// which was the proximate cause of "anchor 2 calibration fails". savedDelay
// is a RAM cache of what's actually committed, so we never re-read flash.
static const uint32_t NVS_COMMIT_MS = 1500;
static uint16_t savedDelay;      // last value written to NVS (RAM cache)
static uint16_t pendingDelay;    // most recent delay applied to the radio
static uint32_t pendingSinceMs = 0;
static bool     pendingDirty   = false;

static void splashIdent(uint16_t antDelay) {
  char title[12], l1[20];
  snprintf(title, sizeof(title), "ANCHOR %02X", ANCHOR_ID);
  snprintf(l1,    sizeof(l1),    "Delay: %u",   antDelay);
  oled.showSplash(title, l1, "Waiting...");
}

void setup() {
  Serial.begin(115200);
  delay(200);
  oled.begin();

  ChargeMode::begin();
  ChargeMode::checkAndSleep(&oled);   // short GPIO13-14 -> charge mode (deep sleep)

  // Load calibrated antenna delay from NVS; fall back to sketch constant.
  prefs.begin("uwb", false);
  uint16_t antDelay = prefs.getUShort("antDelay", ANTENNA_DELAY);
  savedDelay = pendingDelay = antDelay;
  Serial.printf("[CAL] Antenna delay: %u%s\n", antDelay,
                antDelay == ANTENNA_DELAY ? " (factory default)" : " (from NVS)");

  splashIdent(antDelay);
  engine.begin(TWR_ANCHOR, ANCHOR_ID, antDelay);
  engine.printDeviceId();
  Serial.printf("Anchor 0x%02X ready (antenna delay %u)\n", ANCHOR_ID, antDelay);
}

void loop() {
  ChargeMode::checkAndSleep(&oled);   // jumper 13-14 enters charge mode at any time
  bool served = engine.serviceResponder();

  // Persist a host-pushed antenna delay update to NVS so it survives reboots.
  // Debounced: mark dirty on every push, but only WRITE once the delay has
  // settled (no update for NVS_COMMIT_MS) and it actually differs from what's
  // already stored — a HWCALIB search pushes ~15 candidate delays; this turns
  // that into exactly one flash write instead of ~15.
  if (engine.antDelayWasUpdated()) {
    pendingDelay   = engine.antDelay();
    pendingSinceMs = millis();
    pendingDirty   = true;
  }
  if (pendingDirty && (millis() - pendingSinceMs > NVS_COMMIT_MS) && pendingDelay != savedDelay) {
    prefs.putUShort("antDelay", pendingDelay);
    savedDelay   = pendingDelay;
    pendingDirty = false;
    Serial.printf("[CAL] NVS saved antenna delay: %u\n", savedDelay);
  }

  // Update OLED only AFTER a complete TWR exchange (served=true).
  // At that moment the next POLL to this anchor is ≥45 ms away (tag is polling
  // the other anchors), so the ~23 ms I2C refresh cannot block POLL_ACK timing.
  // The 2000 ms fallback keeps the display live when no tag is in range.
  static uint32_t oledLast = 0;
  static bool     oledDue  = false;
  if (served) oledDue = true;
  uint32_t now = millis();
  if (oledDue || now - oledLast > 2000) {
    oledDue  = false;
    oledLast = now;
    char title[12], l1[20], l2[20], l3[20];
    snprintf(title, sizeof(title), "ANCHOR %02X", ANCHOR_ID);
    snprintf(l1,    sizeof(l1),    "Delay: %u",   engine.antDelay());
    if (engine.lastPeer() != UWB_ADDR_INVALID) {
      snprintf(l2, sizeof(l2), "Tag:  0x%02X", engine.lastPeer());
      snprintf(l3, sizeof(l3), "d=%.2fm",      engine.lastDistance());
    } else {
      strncpy(l2, "Waiting...", sizeof(l2));
      l3[0] = '\0';
    }
    oled.showSplash(title, l1, l2, l3);
  }
}
