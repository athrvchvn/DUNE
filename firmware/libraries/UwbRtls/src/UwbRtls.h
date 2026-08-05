/*
 * UwbRtls.h - Umbrella header for the UWB RTLS library.
 *
 * A clean DW1000 ranging + scheduling layer for the Makerfabs "ESP32 UWB Pro
 * with Display". It vendors only the proven low-level DW1000 register driver
 * and replaces the high-level protocol (where the upstream ~4-anchor / single-
 * tag limits live) with:
 *   - DS-TWR (asymmetric, clock-offset cancelling)      -> TwrEngine
 *   - explicit-address round-robin TDMA on the tag      -> UwbScheduler
 *   - multi-tag TDMA over a wired UART link             -> TagLink
 *   - raw-range streaming to a MATLAB host (UDP/Serial)  -> HostLink
 *   - designed-in IMU + OLED hooks                       -> SensorImu, OledStatus
 *
 * Position solving / filtering live on the MATLAB host (see /matlab), so adding
 * anchors never requires a firmware change.
 *
 * NOTE: define your transport (UWB_HOSTLINK_UDP / UWB_HOSTLINK_SERIAL) and,
 * optionally, UWB_USE_OLED BEFORE including this header.
 */
#ifndef UWBRTLS_H
#define UWBRTLS_H

#include "UwbConfig.h"
#include "UwbFrame.h"
#include "TwrEngine.h"
#include "UwbScheduler.h"
#include "TagLink.h"
#include "SensorImu.h"
#include "HostLink.h"
#include "OledStatus.h"
#include "ChargeMode.h"

#endif // UWBRTLS_H
