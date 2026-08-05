#include "UwbConfig.h"
#include "UwbFrame.h"
#include <string.h>

uint8_t writeHeader(byte* f, uint8_t type, uint8_t src, uint8_t dst, uint8_t seq) {
  f[0] = type;
  f[1] = src;
  f[2] = dst;
  f[3] = seq;
  return UWB_HDR_LEN;
}

void packRangePayload(byte* f, const DW1000Time& pollSent,
                      const DW1000Time& pollAckReceived,
                      const DW1000Time& rangeSent) {
  // Each DW1000Time serializes to 5 bytes (40-bit device-time timestamp).
  pollSent.getTimestamp(f + UWB_HDR_LEN + 0 * UWB_TS_LEN);
  pollAckReceived.getTimestamp(f + UWB_HDR_LEN + 1 * UWB_TS_LEN);
  rangeSent.getTimestamp(f + UWB_HDR_LEN + 2 * UWB_TS_LEN);
}

void unpackRangePayload(const byte* f, DW1000Time& pollSent,
                        DW1000Time& pollAckReceived,
                        DW1000Time& rangeSent) {
  // setTimestamp(byte[]) reads 5 bytes. Cast away const: the driver API takes a
  // non-const pointer but does not modify the buffer.
  byte* p = const_cast<byte*>(f);
  pollSent.setTimestamp(p + UWB_HDR_LEN + 0 * UWB_TS_LEN);
  pollAckReceived.setTimestamp(p + UWB_HDR_LEN + 1 * UWB_TS_LEN);
  rangeSent.setTimestamp(p + UWB_HDR_LEN + 2 * UWB_TS_LEN);
}

void packReportPayload(byte* f, float distanceMeters, float rxPowerDbm) {
  memcpy(f + UWB_HDR_LEN + 0, &distanceMeters, sizeof(float));
  memcpy(f + UWB_HDR_LEN + 4, &rxPowerDbm,     sizeof(float));
}

void unpackReportPayload(const byte* f, float& distanceMeters, float& rxPowerDbm) {
  memcpy(&distanceMeters, f + UWB_HDR_LEN + 0, sizeof(float));
  memcpy(&rxPowerDbm,     f + UWB_HDR_LEN + 4, sizeof(float));
}

void packSurveyResp(byte* f, uint8_t target, float distMeters, float rxPowerDbm) {
  f[UWB_HDR_LEN] = target;
  memcpy(f + UWB_HDR_LEN + 1, &distMeters,   sizeof(float));
  memcpy(f + UWB_HDR_LEN + 5, &rxPowerDbm,   sizeof(float));
}

void unpackSurveyResp(const byte* f, uint8_t& target,
                      float& distMeters, float& rxPowerDbm) {
  target = f[UWB_HDR_LEN];
  memcpy(&distMeters,  f + UWB_HDR_LEN + 1, sizeof(float));
  memcpy(&rxPowerDbm,  f + UWB_HDR_LEN + 5, sizeof(float));
}

void packAntDelay(byte* f, uint16_t delayTicks) {
  memcpy(f + UWB_HDR_LEN, &delayTicks, sizeof(uint16_t));
}

uint16_t unpackAntDelay(const byte* f) {
  uint16_t v;
  memcpy(&v, f + UWB_HDR_LEN, sizeof(uint16_t));
  return v;
}
