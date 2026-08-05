function s = parseRtlsLine(line)
%PARSERTLSLINE Parse one live "RTLS" ASCII line from the tag firmware.
%   s = dune.parseRtlsLine(line)
%
%   Formats (HostLink.h):
%     v1: RTLS,v1,<t_ms>,<tag_id>,<n>,{<id>,<d_mm>,<rx_dbm>}*n
%     v2: RTLS,v2,<t_ms>,<tag_id>,<n>,{<id>,<d_mm>,<rx_dbm>,<fp_dbm>,<q>}*n
%     v3: RTLS,v3,<t_ms>,<tag_id>,<n>,{<id>,<d_mm>,<rx_dbm>,<fp_dbm>,<q>}*n
%     v4: RTLS,v4,<t_ms>,<tag_id>,<n>,{...v3 fields...,<cfo>,<tex>}*n
%             [,DIAG,<dieTempC>,<vbatV>]
%     both: [,IMU,<status>,<qw>,<qx>,<qy>,<qz>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>]
%
%   v1/v2 are accepted so this repo's pre-DUNE logs still replay. v1 carries no
%   first-path power, so s.fp/s.qual stay NaN -- NLOS gap weighting degrades to
%   unity weights rather than silently scoring against a missing field.
%
%   v4 (Phase-C wobble diagnostics) per anchor: cfo = raw DW1000 carrier
%   integrator of the RANGE_REPORT RX, tex = realised exchange start (ms from
%   sweep start). DIAG = tag DW1000 die temperature (C) + Vbat (V).
%
%   Returns [] for non-RTLS lines (survey/calibration chatter). Output:
%     s.tms, s.tag, s.ids [n], s.dist [n] (m), s.rx, s.fp, s.qual
%     s.cfoPpm [n]  per-anchor carrier offset in ppm (NaN on v3)
%     s.tex [n]     realised exchange start ms (NaN on v3)
%     s.tempC, s.vbat   tag die temperature / battery (NaN when absent)
%     s.imu = [] or struct(status, quat[4], acc[3], gyro[3])
%     s.cycle, s.mrxUs  TagLink cycle id / master receive time (NaN when absent)

% Raw carrier integrator -> ppm (Decawave constants, 110 kbps mode, channel 5:
% Fs/2 correction 998.4e6/2/8192/131072 Hz per LSB, carrier 6489.6 MHz; the
% negative sign follows dwt_readcarrierintegrator's convention: positive ppm =
% remote (anchor) crystal runs faster than ours).
CFO_PPM_PER_LSB = (998.4e6/2/8192/131072) * (-1e6/6489.6e6);

s = [];
line = strtrim(line);
if ~startsWith(line, "RTLS,"), return; end
tok = split(string(line), ',');
if numel(tok) < 5, return; end
switch tok(2)
    case "v1", stride = 3;   % id,d_mm,rx  -- no fp/quality on the wire
    case "v2", stride = 5;
    case "v3", stride = 5;
    case "v4", stride = 7;   % + cfo,tex
    otherwise, return;
end

s.tms = double(tok(3));
s.tag = double(tok(4));
n     = double(tok(5));
s.ids = nan(1,n); s.dist = nan(1,n); s.rx = nan(1,n);
s.fp  = nan(1,n); s.qual = nan(1,n);
s.cfoPpm = nan(1,n); s.tex = nan(1,n);
p = 6;
% Refuse a truncated line outright rather than indexing past the end: a clipped
% datagram would otherwise throw here instead of being skipped like any other
% unparseable chatter.
if numel(tok) < 5 + stride * n, s = []; return; end
for k = 1:n
    s.ids(k)  = double(tok(p));
    s.dist(k) = double(tok(p+1)) / 1000;   % mm -> m
    s.rx(k)   = double(tok(p+2));
    if stride >= 5                          % v1 has no fp/quality fields
        s.fp(k)   = double(tok(p+3));
        s.qual(k) = double(tok(p+4));
    end
    if stride == 7
        s.cfoPpm(k) = double(tok(p+5)) * CFO_PPM_PER_LSB;
        s.tex(k)    = double(tok(p+6));
    end
    p = p + stride;
end

s.tempC = NaN; s.vbat = NaN;
if p <= numel(tok) && tok(p) == "DIAG" && numel(tok) >= p + 2
    s.tempC = double(tok(p+1));
    s.vbat  = double(tok(p+2));
    p = p + 3;
end

s.imu = [];
if p <= numel(tok) && tok(p) == "IMU" && numel(tok) >= p + 11
    v = double(tok(p+1:p+11));
    s.imu = struct('status', v(1), 'quat', v(2:5)', ...
                   'acc', v(6:8)', 'gyro', v(9:11)');
    p = p + 12;
end

% TagLink tails, appended by the MASTER in wired dual-tag mode:
%   ,CYC,<cycle>              on its own lines
%   ,CYC,<cycle>,MRX,<us>     on lines forwarded from the slave
% Both are optional and absent in ring/WiFi mode. MRX is the master's own
% esp_timer clock when the slave's frame landed; s.tms stays the OWNING tag's
% clock, never rewritten, so a join on cycle gives slave-clock, master-receive
% and cycle-start references for the same measurement.
s.cycle = NaN; s.mrxUs = NaN;
if p <= numel(tok) && tok(p) == "CYC" && numel(tok) >= p + 1
    s.cycle = double(tok(p+1));
    p = p + 2;
    if p <= numel(tok) && tok(p) == "MRX" && numel(tok) >= p + 1
        s.mrxUs = double(tok(p+1));
    end
end
end
