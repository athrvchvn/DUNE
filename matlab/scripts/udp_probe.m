function udp_probe(seconds, port)
%UDP_PROBE Verify the tag's UDP broadcast reaches this host (step 7.3, task 1).
%   udp_probe                 % listen 15 s on port 4100
%   udp_probe(30)             % listen 30 s
%   udp_probe(30, 4100)
%
%   Binds a datagram socket and prints every RTLS line that arrives, then a
%   per-tag summary (packets, rate, anchors, IMU?). This is the gate for the
%   whole dual-tag pipeline: if nothing arrives here, the AP is dropping the
%   broadcast (iitk historically blocks client-to-client) — switch to a phone
%   hotspot / dedicated AP before wiring live_dual_tag.
%
%   Uses java.net.DatagramSocket (no Instrument Control Toolbox needed).
%
%   Windows: the first run pops a Windows Defender Firewall prompt for MATLAB —
%   allow it (Private networks) or datagrams are silently discarded.
%
%   Quick network checks if this stays silent:
%     - host and tag on the SAME WiFi (check the tag's OLED / boot banner IP)
%     - firewall allowed inbound UDP for MATLAB
%     - if broadcast is blocked but unicast works: set HOST_IP in the firmware
%       to THIS host's IPv4 (below) and reflash — see the handoff.

arguments
    seconds (1,1) double = 15
    port (1,1) double = 4100
end

fprintf('Listening for UDP datagrams on 0.0.0.0:%d for %.0f s ...\n', port, seconds);
fprintf('(this host IPv4: %s)\n', strjoin(localIPv4s(), ', '));

sock = java.net.DatagramSocket([]);
sock.setReuseAddress(true);
sock.setSoTimeout(200);                    % ms
sock.bind(java.net.InetSocketAddress(int32(port)));
cleanup = onCleanup(@() sock.close());
buf  = int8(zeros(1, 2048));
rpkt = java.net.DatagramPacket(buf, numel(buf));

t0 = tic;
nTotal = 0;
tags = containers.Map('KeyType', 'double', 'ValueType', 'any');
first = true;

while toc(t0) < seconds
    try
        rpkt.setLength(numel(buf));
        sock.receive(rpkt);
    catch
        continue;                          % 200 ms timeout, keep waiting
    end
    len = rpkt.getLength();
    if len <= 0, continue; end
    raw    = char(double(rpkt.getData()).');
    raw    = raw(1:len);
    sender = char(rpkt.getAddress().getHostAddress());
    for line = split(string(raw), newline).'
        ln = strtrim(line);
        if strlength(ln) == 0, continue; end
        nTotal = nTotal + 1;
        if first
            fprintf('\nFIRST PACKET from %s:\n  %s\n\n', sender, ln);
            first = false;
        end
        s = dune.parseRtlsLine(ln);
        if isempty(s), continue; end          % boot/ack chatter
        key = s.tag;
        if ~tags.isKey(key)
            tags(key) = struct('ip', sender, 'n', 0, 't0', toc(t0), ...
                               'tlast', toc(t0), 'nAnch', 0, 'imu', false);
            fprintf('  tag %d (0x%02X) live from %s\n', key, key, sender);
        end
        e = tags(key);
        e.n = e.n + 1; e.tlast = toc(t0);
        e.nAnch = max(e.nAnch, nnz(isfinite(s.dist)));
        e.imu = e.imu || ~isempty(s.imu);
        tags(key) = e;
    end
end

fprintf('\n===== %.0f s: %d line(s) total =====\n', seconds, nTotal);
if tags.Count == 0
    fprintf('NO RTLS packets received. UDP is not getting through.\n');
    fprintf('-> Same WiFi? Firewall allowed? AP blocking broadcast?\n');
    fprintf('   Try a phone hotspot, then re-run. See the function help.\n');
    return;
end
ks = cell2mat(tags.keys);
for key = ks
    e = tags(key);
    span = max(e.tlast - e.t0, 1e-3);
    hz = (e.n - 1) / span;
    fprintf('  tag %3d (0x%02X)  %-15s  %4d pkt  %.1f Hz  up to %d anchors  IMU:%s\n', ...
            key, key, e.ip, e.n, hz, e.nAnch, string(e.imu));
end
if numel(ks) >= 2
    fprintf('\nBOTH tags received over UDP — ready for live_dual_tag.\n');
else
    fprintf('\nOnly one tag heard. Power the second tag (same TAG_RING) and re-run.\n');
end
end

function ips = localIPv4s()
% Best-effort list of this host's IPv4 addresses (to set firmware HOST_IP).
ips = "unknown";
try
    host = char(java.net.InetAddress.getLocalHost().getHostName());
    a = java.net.InetAddress.getAllByName(host);
    out = strings(1, 0);
    for i = 1:numel(a)
        h = char(a(i).getHostAddress());
        if ~contains(h, ':') && ~startsWith(h, "127.")   % IPv4, non-loopback
            out(end+1) = string(h); %#ok<AGROW>
        end
    end
    if ~isempty(out), ips = out; end
catch
end
end
