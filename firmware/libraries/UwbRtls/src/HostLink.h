/*
 * HostLink.h - Stream one range-sweep packet to the MATLAB host.
 *
 * Transport is selected by a COMPILE-TIME flag defined in the sketch BEFORE
 * including this library:
 *     #define UWB_HOSTLINK_UDP      // WiFi/UDP to the host
 *  or #define UWB_HOSTLINK_SERIAL   // USB serial (default if neither defined)
 *
 * This class is header-only on purpose: Arduino compiles library .cpp files in
 * their own translation units, where a #define from the .ino is NOT visible.
 * Being header-only, HostLink is compiled inside the sketch's translation unit,
 * so the sketch's flag actually controls which transport is built.
 *
 * Wire format (versioned ASCII line, trivial bandwidth, easy parse):
 *   RTLS,v3,<t_ms>,<tag_id>,<n>,<id1>,<d1_mm>,<rx1_dbm>,<fp1_dbm>,<q1>,...\n
 *        [,IMU,<status>,<qw>,<qx>,<qy>,<qz>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>]
 *   - v3 adds BNO085 accuracy status byte and gyroscope (gx,gy,gz) to IMU tail.
 *   - v2 (7-field IMU tail) is still accepted by the Python parser (backward compat).
 *   - v1 (3 fields/anchor) is accepted by the Python FrameParser for compatibility.
 *   - The IMU,... tail is appended only when an IMU sample is present.
 */
#ifndef UWBRTLS_HOSTLINK_H
#define UWBRTLS_HOSTLINK_H

#include <Arduino.h>
#include <math.h>
#include "UwbScheduler.h"
#include "SensorImu.h"

// Default to serial if the sketch did not pick a transport.
#if !defined(UWB_HOSTLINK_UDP) && !defined(UWB_HOSTLINK_SERIAL)
#define UWB_HOSTLINK_SERIAL
#endif

#if defined(UWB_HOSTLINK_UDP)
#include <WiFi.h>
#include <WiFiUdp.h>
#endif

class HostLink {
public:
#if defined(UWB_HOSTLINK_UDP)
  // Connect to WiFi and target the MATLAB host's UDP port.
  void begin(const char* ssid, const char* pass, IPAddress host, uint16_t port,
             unsigned long serialBaud = 115200) {
    Serial.begin(serialBaud);
    _host = host;
    _port = port;
    _ssid = ssid;
    _pass = pass;
    _wifiConnect();
  }
  // Reconnect WiFi if it has dropped. Called lazily from sendSweep/sendRaw.
  void checkWifi() {
    if (WiFi.status() != WL_CONNECTED) {
      uint32_t now = millis();
      if (now - _lastReconnectMs < 10000) return;   // back off: try every 10 s
      _lastReconnectMs = now;
      Serial.println(F("[WIFI] disconnected — reconnecting..."));
      _wifiConnect();
    }
  }
#else
  void begin(unsigned long serialBaud = 115200) {
    Serial.begin(serialBaud);
  }
#endif

  // Receive an incoming command from the host (UDP only; no-op for serial).
  // Returns number of bytes read (0 if no packet waiting); buf is null-terminated.
  // The host sends calibration commands (SETANTDELAY, etc.) to the tag's IP on
  // the same port the tag streams to — the socket is already bound for receive.
  uint8_t receiveCmd(char* buf, size_t maxLen) {
#if defined(UWB_HOSTLINK_UDP)
    int n = _udp.parsePacket();
    if (n <= 0) return 0;
    n = _udp.read(buf, (int)(maxLen - 1));
    if (n < 0) n = 0;
    while (n > 0 && (buf[n-1] == '\n' || buf[n-1] == '\r')) n--;  // strip trailing newline
    buf[n] = '\0';
    return (uint8_t)n;
#else
    return 0;
#endif
  }

  // Send a pre-formatted line as-is (survey control lines, diagnostics, etc.).
  // Caller must include the trailing '\n'.
  void sendRaw(const char* line) {
    sendLine(line, (int)strlen(line));
  }

  // Send one pre-built line as ONE UDP datagram (the Python reader treats each
  // datagram as a single line), mirrored to serial. Used by the master to
  // forward the slave's formatted sweep separately from its own.
  void sendLine(const char* line, int len) {
    if (len <= 0) return;
#if defined(UWB_HOSTLINK_UDP)
    checkWifi();
    if (WiFi.status() == WL_CONNECTED) {
      _udp.beginPacket(_host, _port);
      _udp.write(reinterpret_cast<const uint8_t*>(line), (size_t)len);
      _udp.endPacket();
    }
#endif
    Serial.write(line, len);
  }

  // Format and send one sweep. imu may be nullptr (or invalid) to omit IMU data.
  void sendSweep(uint32_t tMs, uint8_t tagId, const UwbScheduler& sched,
                 const ImuSample* imu = nullptr) {
    char buf[768];
    int len = format(buf, sizeof(buf), tMs, tagId, sched, imu);
    if (len <= 0) return;

#if defined(UWB_HOSTLINK_UDP)
    checkWifi();
    if (WiFi.status() == WL_CONNECTED) {
      _udp.beginPacket(_host, _port);
      _udp.write(reinterpret_cast<const uint8_t*>(buf), len);
      _udp.endPacket();
      _udpDrops = 0;
    } else {
      _udpDrops++;
      if (_udpDrops == 1 || (_udpDrops & 0x3F) == 0)
        Serial.printf("[WIFI] UDP drop #%u (not connected) host=%d.%d.%d.%d\n",
                      _udpDrops, _host[0], _host[1], _host[2], _host[3]);
    }
    // Mirror to serial too, handy while debugging.
    Serial.write(buf, len);
#else
    Serial.write(buf, len);
#endif
  }

  // Build an RTLS,v3 line from a raw results array; returns bytes written
  // (incl. '\n') or -1. THE single binary->ASCII serializer: the master uses
  // it both for its own sweep and for the slave's TagLink DATA records, so the
  // two tags' lines can never drift apart in format. Only valid entries are
  // emitted; tMs is the OWNING tag's clock (never rewrite a forwarded one).
  static int format(char* buf, size_t size, uint32_t tMs, uint8_t tagId,
                    const RangeResult* results, uint8_t n,
                    const ImuSample* imu = nullptr) {
    uint8_t nValid = 0;
    for (uint8_t i = 0; i < n; i++)
      if (results[i].valid) nValid++;

    int p = snprintf(buf, size, "RTLS,v3,%lu,%u,%u",
                     (unsigned long)tMs, (unsigned)tagId, (unsigned)nValid);
    if (p < 0 || (size_t)p >= size) return -1;

    for (uint8_t i = 0; i < n; i++) {
      const RangeResult& r = results[i];
      if (!r.valid) continue;
      long  mm = lround(r.distance * 1000.0f);
      int   q  = (int)lround(r.rxPower);
      int w = snprintf(buf + p, size - p, ",%u,%ld,%d,%.1f,%.2f",
                       (unsigned)r.id, mm, q, r.fpPower, r.quality);
      if (w < 0 || (size_t)(p + w) >= size) return -1;
      p += w;
    }

    if (imu && imu->valid) {
      int w = snprintf(buf + p, size - p,
                       ",IMU,%u,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f",
                       (unsigned)imu->status,
                       imu->qw, imu->qx, imu->qy, imu->qz,
                       imu->ax, imu->ay, imu->az,
                       imu->gx, imu->gy, imu->gz);
      if (w < 0 || (size_t)(p + w) >= size) return -1;
      p += w;
    }

    if ((size_t)(p + 1) >= size) return -1;
    buf[p++] = '\n';
    return p;
  }

  // Insert a printf-style tail BEFORE the trailing '\n' of a formatted line
  // (e.g. ",CYC,%u" or ",CYC,%u,MRX,%llu"). Returns the new length, or the
  // old length unchanged if it would not fit.
  static int appendTail(char* buf, size_t size, int len, const char* fmt, ...) {
    if (len <= 0 || buf[len - 1] != '\n') return len;
    va_list ap;
    va_start(ap, fmt);
    int w = vsnprintf(buf + len - 1, size - (size_t)len, fmt, ap);
    va_end(ap);
    if (w < 0 || (size_t)(len - 1 + w + 1) >= size) { buf[len - 1] = '\n'; return len; }
    buf[len - 1 + w] = '\n';
    return len + w;
  }

  // Scheduler-based overload (a tag's own sweep) delegating to the array one.
  static int format(char* buf, size_t size, uint32_t tMs, uint8_t tagId,
                    const UwbScheduler& sched, const ImuSample* imu = nullptr) {
    RangeResult results[UWB_MAX_ANCHORS];
    uint8_t n = sched.anchorCount();
    if (n > UWB_MAX_ANCHORS) n = UWB_MAX_ANCHORS;
    for (uint8_t i = 0; i < n; i++) results[i] = sched.result(i);
    return format(buf, size, tMs, tagId, results, n, imu);
  }

private:

#if defined(UWB_HOSTLINK_UDP)
  WiFiUDP      _udp;
  IPAddress    _host;
  uint16_t     _port          = 0;
  const char*  _ssid          = nullptr;
  const char*  _pass          = nullptr;
  uint32_t     _lastReconnectMs = 0;
  uint32_t     _udpDrops      = 0;

  void _wifiConnect() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(_ssid, _pass);
    Serial.printf("[WIFI] connecting to \"%s\"...", _ssid);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
      delay(250);
      Serial.print('.');
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("[WIFI] connected  IP=%s  host=%d.%d.%d.%d:%u\n",
                    WiFi.localIP().toString().c_str(),
                    _host[0], _host[1], _host[2], _host[3], _port);
      _udp.begin(_port);
      _udpDrops = 0;
    } else {
      Serial.println(F("[WIFI] FAILED — UDP packets will be dropped until reconnect"));
    }
    _lastReconnectMs = millis();
  }
#endif
};

#endif // UWBRTLS_HOSTLINK_H
