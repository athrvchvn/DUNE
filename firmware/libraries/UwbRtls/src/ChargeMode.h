/*
 * ChargeMode.h - Jumper-detected deep-sleep "charge mode".
 *
 * Short PIN_CHARGE_SENSE to PIN_CHARGE_DRIVE (default GPIO 13 <-> 14) to put the
 * board into a near-zero-activity state while its battery charges: the DW1000
 * radio is stopped (held in reset / left idle - never ranging), WiFi is turned
 * off, and the ESP32 enters deep sleep (~10 uA), waking only briefly every
 * CHARGE_RECHECK_SEC seconds to re-test the jumper. Remove the jumper and the
 * board resets back into normal operation within a few seconds. This stops the
 * module from cycling the battery (radio bursts / WiFi / PA-LNA / IMU) while it
 * is on charge, which preserves battery lifespan.
 *
 * Detection is robust: jumperPresent() drives the DRIVE pin LOW then HIGH and
 * requires the SENSE pin to follow both (a genuine short), so a pin merely stuck
 * at one level can NOT trigger charge mode. Neither pin is an ESP32 strapping
 * pin, so this never interferes with flashing or boot mode.
 *
 * Usage in every sketch:
 *   setup():  oled.begin();                      // (if the board has an OLED)
 *             ChargeMode::begin();
 *             ChargeMode::checkAndSleep(&oled);   // BEFORE WiFi / DW1000 init
 *             ... normal init ...
 *   loop():   ChargeMode::checkAndSleep(&oled);   // allows plugging in while running
 *             ... normal work ...
 *
 * Override the pins (BEFORE including <UwbRtls.h>) if 13/14 are taken:
 *   #define PIN_CHARGE_SENSE 35
 *   #define PIN_CHARGE_DRIVE 33
 *
 * Note: the BNO085 IMU on the WROVER tag has no power-gate pin, so it keeps its
 * own small idle draw during charge mode; the dominant savings (ESP32 deep
 * sleep, WiFi off, radio not ranging) are still achieved.
 */
#ifndef UWBRTLS_CHARGEMODE_H
#define UWBRTLS_CHARGEMODE_H

#include <Arduino.h>
#include <esp_sleep.h>
#include "UwbConfig.h"
#include "OledStatus.h"
#if defined(UWB_HOSTLINK_UDP)
#include <WiFi.h>
#endif

#ifndef PIN_CHARGE_SENSE
#define PIN_CHARGE_SENSE 13   // INPUT_PULLUP; reads the DRIVE pin level when shorted
#endif
#ifndef PIN_CHARGE_DRIVE
#define PIN_CHARGE_DRIVE 14   // driven OUTPUT to probe for the short
#endif
#ifndef CHARGE_RECHECK_SEC
#define CHARGE_RECHECK_SEC 3  // deep-sleep re-check interval while charging (s)
#endif

namespace ChargeMode {

// Configure the detection pins. Call once in setup() before checkAndSleep().
inline void begin() {
  pinMode(PIN_CHARGE_DRIVE, OUTPUT);
  digitalWrite(PIN_CHARGE_DRIVE, LOW);
  pinMode(PIN_CHARGE_SENSE, INPUT_PULLUP);
}

// Debounced, two-phase short detection. Returns true only if SENSE faithfully
// follows DRIVE both LOW and HIGH across several reads (a real jumper) - this
// rejects a pin that is simply stuck at one level.
inline bool jumperPresent() {
  const uint8_t TRIES = 4;
  for (uint8_t i = 0; i < TRIES; i++) {
    digitalWrite(PIN_CHARGE_DRIVE, LOW);
    delayMicroseconds(80);
    if (digitalRead(PIN_CHARGE_SENSE) != LOW)  { digitalWrite(PIN_CHARGE_DRIVE, LOW); return false; }
    digitalWrite(PIN_CHARGE_DRIVE, HIGH);
    delayMicroseconds(80);
    if (digitalRead(PIN_CHARGE_SENSE) != HIGH) { digitalWrite(PIN_CHARGE_DRIVE, LOW); return false; }
  }
  digitalWrite(PIN_CHARGE_DRIVE, LOW);
  return true;
}

// If the jumper is present: stop the radio, turn WiFi off, show CHARGING, and
// deep-sleep (re-checking every CHARGE_RECHECK_SEC). Does not return while the
// jumper is in place; on removal the ESP32 resets into normal operation. No-op
// when the jumper is absent, so it is safe to call every loop().
inline void checkAndSleep(OledStatus* oled = nullptr) {
  if (!jumperPresent()) return;

  // Radio off: hold the DW1000 in reset (best effort). Even if the line floats
  // during deep sleep the chip only settles to idle - it never ranges.
  pinMode(UWB_PIN_RST, OUTPUT);
  digitalWrite(UWB_PIN_RST, LOW);

#if defined(UWB_HOSTLINK_UDP)
  WiFi.disconnect(true);     // wifioff=true (keeps stored credentials)
  WiFi.mode(WIFI_OFF);
#endif

  Serial.println(F("[CHARGE] Jumper 13-14 detected -> charge mode (deep sleep). Remove to resume."));
  Serial.flush();
  if (oled) oled->showSplash("CHARGING", "Battery charge", "Remove jumper", "13-14 to resume");

  esp_sleep_enable_timer_wakeup((uint64_t)CHARGE_RECHECK_SEC * 1000000ULL);
  esp_deep_sleep_start();   // resets on wake; setup() re-evaluates the jumper
}

}  // namespace ChargeMode

#endif // UWBRTLS_CHARGEMODE_H
