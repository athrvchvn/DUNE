/*
 * UwbConfig.h - Shared radio / board configuration for the UwbRtls library.
 *
 * These settings MUST be identical on every board in the network (tag + all
 * anchors), with the sole exception of per-device values that each sketch sets
 * itself: the device short address and the calibrated antenna delay.
 *
 * Board: Makerfabs "ESP32 UWB Pro with Display" (Decawave DW1000 / BU01).
 */
#ifndef UWBRTLS_UWBCONFIG_H
#define UWBRTLS_UWBCONFIG_H

#include <Arduino.h>
#include "dw1000/DW1000.h"

// ---------------------------------------------------------------------------
// Pin map defaults — Makerfabs ESP32 UWB Pro with Display.
//
// Board variants use different CS (SS) pins:
//   ESP32 UWB Pro WITH display  : UWB_PIN_SS = 21  (default below)
//   ESP32 UWB  (no Pro/display) : UWB_PIN_SS = 4
//
// A sketch can override any pin BEFORE including UwbRtls.h:
//   #define UWB_PIN_SS 4
//   #include <UwbRtls.h>
// ---------------------------------------------------------------------------
#ifndef UWB_PIN_SCK
#define UWB_PIN_SCK   18
#endif
#ifndef UWB_PIN_MISO
#define UWB_PIN_MISO  19
#endif
#ifndef UWB_PIN_MOSI
#define UWB_PIN_MOSI  23
#endif
#ifndef UWB_PIN_SS
#define UWB_PIN_SS    21   // SPI chip select — override to 4 for non-Pro boards
#endif
#ifndef UWB_PIN_RST
#define UWB_PIN_RST   27   // DW1000 reset
#endif
#ifndef UWB_PIN_IRQ
#define UWB_PIN_IRQ   34   // DW1000 interrupt (input-only pin, fine for IRQ)
#endif

#ifndef OLED_PIN_SDA
#define OLED_PIN_SDA   4
#endif
#ifndef OLED_PIN_SCL
#define OLED_PIN_SCL   5
#endif

// ---------------------------------------------------------------------------
// Radio profile - range accuracy mode. Same on every board. (Phase 1.0)
//   MODE_LONGDATA_RANGE_ACCURACY = 110 kbps, 64 MHz PRF, 2048-symbol preamble.
//   64 MHz PRF sharpens the CIR autocorrelation peak and rejects multipath
//   better than the 16 MHz LOWPOWER variants. Long preamble maximises
//   sensitivity at the cost of ~10 Hz sweep rate.
//   Phase 1.3B (MODE_SHORTDATA_FAST_ACCURACY, 6.8 Mb/s) was attempted but
//   reverted due to hardware incompatibility — revisit after diagnosis.
// ---------------------------------------------------------------------------
#define UWB_RADIO_MODE   DW1000.MODE_LONGDATA_RANGE_ACCURACY
#define UWB_CHANNEL      DW1000.CHANNEL_5

// Reply delay used for the delayed transmits in two-way ranging (microseconds).
// History: 7000 us (16 MHz LOWPOWER) → 6000 us (Phase 1.0, 64 MHz ACCURACY) →
//          5000 us (Phase 1.3A, tighter margin, well-validated).
// At 110 kbps the frame takes ~2 ms on-air; ESP32 SPI + DW1000 processing
// needs ~1000-1200 us; 5000 us leaves a comfortable ~1800 us margin.
#define UWB_REPLY_DELAY_US   5000

// Default antenna delay (DW1000 ticks). Each board overrides this with its own
// CALIBRATED value (see examples/AntennaCalibration). 16384 is the chip reset
// default; tuned values typically land in 16450..16650.
#define UWB_DEFAULT_ANTENNA_DELAY  16384

// ---------------------------------------------------------------------------
// Addressing (1-byte short addresses, our own scheme - not 802.15.4).
//   0x00          : reserved / invalid
//   0x01 .. 0xEF  : anchors
//   0xF0 .. 0xFE  : tags
//   0xFF          : broadcast (reserved for future multi-tag announce/slotting)
// ---------------------------------------------------------------------------
#define UWB_ADDR_INVALID    0x00
#define UWB_ADDR_BROADCAST  0xFF
#define UWB_ADDR_TAG_BASE   0xF0   // first tag = 0xF0

// Network id (shared). Frame filtering is done in software, so this is mostly
// cosmetic, but we keep it consistent across the fleet.
#define UWB_NETWORK_ID  0xDECA

// ---------------------------------------------------------------------------
// TagLink — wired multi-tag TDMA (master/slave tags over UART2, TagLink.h).
// The master tag owns the schedule: it sweeps, then grants the slave its air
// slot over the wire; each tag's radio is TRXOFF during the other's slot.
// UART payloads are packed binary; ASCII exists only on the host link.
//
// Wiring (both boards): TAGLINK_PIN_TX -> peer's TAGLINK_PIN_RX, plus common
// GND. GPIO 22 (out) / GPIO 35 (input-only, fine for RX) are free on both the
// TagWrover map (SS=4, OLED 32/33, BNO 25/26) and the display-board map
// (SS=21, OLED 4/5). A sketch may override any value before <UwbRtls.h>.
// ---------------------------------------------------------------------------
#ifndef TAGLINK_PIN_TX
#define TAGLINK_PIN_TX  22
#endif
#ifndef TAGLINK_PIN_RX
#define TAGLINK_PIN_RX  35
#endif
#ifndef TAGLINK_BAUD
#define TAGLINK_BAUD    921600   // 50 cm wire: comfortable; ~11 µs per byte
#endif
// Slot budget the master allows the slave per cycle. Adaptive at runtime:
// clamp(1.5 x EMA(slave sweep), FLOOR, CEIL); INIT is used until the first
// measurement arrives. CEIL also bounds worst-case master stall on slave death.
#ifndef TAGLINK_BUDGET_INIT_MS
#define TAGLINK_BUDGET_INIT_MS   700
#endif
#ifndef TAGLINK_BUDGET_FLOOR_MS
#define TAGLINK_BUDGET_FLOOR_MS  150
#endif
#ifndef TAGLINK_BUDGET_CEIL_MS
#define TAGLINK_BUDGET_CEIL_MS   700
#endif
// Slave HELLO cadence: fast until the master ACKs (quick hot-plug join), then
// a slow heartbeat. Master declares the slave absent after ABSENT_AFTER
// consecutive slot timeouts and skips its slot until the next HELLO.
#ifndef TAGLINK_HELLO_FAST_MS
#define TAGLINK_HELLO_FAST_MS    100
#endif
#ifndef TAGLINK_HELLO_BEAT_MS
#define TAGLINK_HELLO_BEAT_MS    500
#endif
#ifndef TAGLINK_ABSENT_AFTER
#define TAGLINK_ABSENT_AFTER     3
#endif

#endif // UWBRTLS_UWBCONFIG_H
