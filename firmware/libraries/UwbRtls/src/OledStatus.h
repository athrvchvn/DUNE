/*
 * OledStatus.h - Optional 1.3" SSD1306 status display (I2C on pins 4/5).
 *
 * Header-only and OFF by default so the library compiles with no extra
 * dependencies. To use the on-board display, in your sketch (BEFORE including
 * the library) add:
 *     #define UWB_USE_OLED
 * and install the "Adafruit SSD1306" + "Adafruit GFX" libraries.
 *
 * When UWB_USE_OLED is not defined, every method is a no-op, so sketches can
 * call OledStatus freely regardless of build configuration.
 */
#ifndef UWBRTLS_OLEDSTATUS_H
#define UWBRTLS_OLEDSTATUS_H

#include <Arduino.h>
#include "UwbConfig.h"

#if defined(UWB_USE_OLED)
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

class OledStatus {
public:
  bool begin() {
    Wire.begin(OLED_PIN_SDA, OLED_PIN_SCL);
    // Try both common SSD1306 addresses.
    _ok = _display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    if (!_ok) _ok = _display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
    if (_ok) {
      _display.clearDisplay();
      _display.setTextColor(SSD1306_WHITE);
      _display.display();
    }
    return _ok;
  }

  // Up to 4 small lines (text size 1, 8 px each).
  void show(const char* l0, const char* l1 = "",
            const char* l2 = "", const char* l3 = "") {
    if (!_ok) return;
    _display.clearDisplay();
    _display.setTextSize(1);
    _display.setCursor(0, 0);   _display.println(l0);
    _display.setCursor(0, 16);  _display.println(l1);
    _display.setCursor(0, 32);  _display.println(l2);
    _display.setCursor(0, 48);  _display.println(l3);
    _display.display();
  }

  // Large title (text size 2) + up to 3 small detail lines.
  // Use for role identification: showSplash("ANCHOR 01", "Dly:16305", ...)
  void showSplash(const char* title, const char* l1 = "",
                  const char* l2 = "", const char* l3 = "") {
    if (!_ok) return;
    _display.clearDisplay();
    _display.setTextSize(2);
    _display.setCursor(0, 0);  _display.println(title);
    _display.setTextSize(1);
    _display.setCursor(0, 20); _display.println(l1);
    _display.setCursor(0, 32); _display.println(l2);
    _display.setCursor(0, 44); _display.println(l3);
    _display.display();
  }

private:
  Adafruit_SSD1306 _display{128, 64, &Wire, -1};
  bool _ok = false;
};

#else  // ---- no-op stub ----

class OledStatus {
public:
  bool begin() { return false; }
  void show(const char* = "", const char* = "",
            const char* = "", const char* = "") {}
  void showSplash(const char* = "", const char* = "",
                  const char* = "", const char* = "") {}
};

#endif // UWB_USE_OLED

#endif // UWBRTLS_OLEDSTATUS_H
