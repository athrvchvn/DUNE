/*
 * UwbFrame.h - Minimal application framing for the UwbRtls protocol.
 *
 * We do NOT use 802.15.4 MAC framing (that, plus the auto-discovery state
 * machine, is exactly where the upstream library's 4-anchor / single-tag
 * limits live). Instead every UWB payload starts with a tiny fixed header and
 * the receiver filters by destination address in software, so the number of
 * anchors is bounded only by your address space, not by the protocol.
 *
 *   byte[0] = message type   (UwbMsgType)
 *   byte[1] = source short address
 *   byte[2] = destination short address (UWB_ADDR_BROADCAST = 0xFF)
 *   byte[3] = sequence number
 *   byte[4..] = type-specific payload
 *
 * The DW1000's hardware frame-check (2-byte CRC) is left enabled, so corrupt
 * frames are dropped by the radio before we ever see them.
 */
#ifndef UWBRTLS_UWBFRAME_H
#define UWBRTLS_UWBFRAME_H

#include <Arduino.h>
#include "dw1000/DW1000Time.h"

// Message types (kept numerically compatible with the classic POLL/RANGE flow).
enum UwbMsgType : uint8_t {
  MSG_POLL         = 0,  // tag  -> anchor : start of an exchange
  MSG_POLL_ACK     = 1,  // anchor -> tag  : acknowledge
  MSG_RANGE        = 2,  // tag  -> anchor : carries tag's 3 timestamps
  MSG_RANGE_REPORT = 3,  // anchor -> tag  : carries computed distance + rx power
  MSG_RANGE_FAILED = 255,
  // Types 10/11 were the retired over-air token ring (ANNOUNCE/SLOT_GRANT);
  // multi-tag coordination now runs over the wired TagLink UART. Keep 10/11
  // unassigned so mixed-firmware fleets never misread stale frames.
  // Anchor self-survey (Phase 1.5): tag asks an anchor to range to another anchor.
  MSG_SURVEY_REQ   = 0x50, // tag -> anchor_A : "range to <target>"
  MSG_SURVEY_RESP  = 0x51, // anchor_A -> tag : "<target>, dist, rxp"
  // Antenna delay push (calibration): host->tag->anchor, stored in NVS.
  MSG_ANT_DELAY     = 0x52, // tag -> anchor : new antenna delay value (uint16_t)
  MSG_ANT_DELAY_ACK = 0x53, // anchor -> tag : echo applied delay (uint16_t)
};

// Header / payload layout.
static const uint8_t UWB_HDR_LEN        = 4;   // type, src, dst, seq
static const uint8_t UWB_TS_LEN         = 5;   // one DW1000 40-bit timestamp
static const uint8_t UWB_FRAME_MAXLEN   = 32;  // generous upper bound

// RANGE payload = 3 timestamps (pollSent, pollAckReceived, rangeSent).
static const uint8_t UWB_RANGE_PAYLOAD_LEN = 3 * UWB_TS_LEN;          // 15
static const uint8_t UWB_RANGE_LEN         = UWB_HDR_LEN + UWB_RANGE_PAYLOAD_LEN; // 19

// RANGE_REPORT payload = float distance(m) + float rx power(dBm).
static const uint8_t UWB_REPORT_PAYLOAD_LEN = 2 * sizeof(float);      // 8
static const uint8_t UWB_REPORT_LEN         = UWB_HDR_LEN + UWB_REPORT_PAYLOAD_LEN; // 12

// SURVEY_REQ payload = 1 byte (target anchor address).
static const uint8_t UWB_SURVEY_REQ_LEN  = UWB_HDR_LEN + 1;           // 5

// SURVEY_RESP payload = target(1) + dist_m(4) + rxPower(4).
static const uint8_t UWB_SURVEY_RESP_LEN = UWB_HDR_LEN + 1 + 2 * sizeof(float); // 13

// ANT_DELAY / ANT_DELAY_ACK payload = uint16_t delay ticks (2 bytes).
static const uint8_t UWB_ANT_DELAY_LEN   = UWB_HDR_LEN + sizeof(uint16_t);      // 6

// --- header accessors -------------------------------------------------------
inline uint8_t frameType(const byte* f) { return f[0]; }
inline uint8_t frameSrc (const byte* f) { return f[1]; }
inline uint8_t frameDst (const byte* f) { return f[2]; }
inline uint8_t frameSeq (const byte* f) { return f[3]; }

// Build the 4-byte header into f. Returns bytes written (UWB_HDR_LEN).
uint8_t writeHeader(byte* f, uint8_t type, uint8_t src, uint8_t dst, uint8_t seq);

// True if this frame is addressed to us (exact match or broadcast).
inline bool frameIsForUs(const byte* f, uint8_t myAddr) {
  return frameDst(f) == myAddr || frameDst(f) == UWB_ADDR_BROADCAST;
}

// --- RANGE payload pack/unpack (3 timestamps) ------------------------------
void packRangePayload(byte* f, const DW1000Time& pollSent,
                      const DW1000Time& pollAckReceived,
                      const DW1000Time& rangeSent);
void unpackRangePayload(const byte* f, DW1000Time& pollSent,
                        DW1000Time& pollAckReceived,
                        DW1000Time& rangeSent);

// --- RANGE_REPORT payload pack/unpack --------------------------------------
void packReportPayload(byte* f, float distanceMeters, float rxPowerDbm);
void unpackReportPayload(const byte* f, float& distanceMeters, float& rxPowerDbm);

// --- SURVEY payload pack/unpack --------------------------------------------
inline void    packSurveyReq(byte* f, uint8_t target) { f[UWB_HDR_LEN] = target; }
inline uint8_t unpackSurveyReqTarget(const byte* f)   { return f[UWB_HDR_LEN]; }

void packSurveyResp(byte* f, uint8_t target, float distMeters, float rxPowerDbm);
void unpackSurveyResp(const byte* f, uint8_t& target,
                      float& distMeters, float& rxPowerDbm);

// --- ANT_DELAY / ANT_DELAY_ACK payload pack/unpack -------------------------
void     packAntDelay(byte* f, uint16_t delayTicks);
uint16_t unpackAntDelay(const byte* f);

#endif // UWBRTLS_UWBFRAME_H
