classdef TagUdp < handle
    %TAGUDP Line-based UDP link to DUNE tag(s) — the dual-tag transport.
    %   Drop-in sibling of dune.TagSerial with the SAME interface
    %   (start/drain/drainEvents/stop), so the whole downstream sweep path is
    %   unchanged. Binds one socket to the port the firmware streams to (HostLink
    %   UDP: broadcast to 255.255.255.255:4100) and ingests every RTLS line from
    %   ALL tags at once. Ingest stays protocol- AND tag-agnostic: the consumer
    %   only ever sees sweep structs and demuxes by s.tag.
    %
    %   tu = dune.TagUdp()             % bind 0.0.0.0:4100 (all interfaces)
    %   tu = dune.TagUdp(4100)
    %   tu.start()                     % open the socket
    %   sweeps = tu.drain()            % pumps the socket, returns parsed sweeps
    %   events = tu.drainEvents()      % non-RTLS lines (boot, MYDELAY_ACK, ...)
    %   tu.sendCmd("SETMYDELAY,240,16360", 240)  % unicast a command back to tag 240
    %   tu.stop()
    %
    %   Uses java.net.DatagramSocket (ships with MATLAB's JVM) — NOT udpport,
    %   which needs the Instrument Control Toolbox we do not have. There is no
    %   background callback: drain()/drainEvents() pump the socket on demand,
    %   which matches live_tag's per-loop drain pattern. Call drain() often
    %   enough that the OS receive buffer does not overflow (a few Hz is plenty).
    %
    %   Each parsed sweep is the dune.parseRtlsLine struct plus .thost (posix s,
    %   host receive time) and .srcIp (sender). The tag's source IP is learned
    %   from its stream and remembered per tag_id, so sendCmd() can unicast a
    %   command straight back to the right tag (the firmware listens for UDP
    %   commands on the same port it broadcasts from — see HostLink.receiveCmd).
    %
    %   Set .rawLogFid (an fopen'd fid) BEFORE start() to tee every incoming
    %   line, tab-stamped with t_host and sender IP, into a raw capture file.
    %
    %   NOTE (Windows): the first bind prompts Windows Defender Firewall to allow
    %   inbound UDP for MATLAB — allow it, or the broadcast never arrives. If UDP
    %   is blocked on the AP (iitk drops client-to-client/broadcast), no socket
    %   option helps; use a phone hotspot / dedicated AP. Verify with udp_probe.

    properties
        port = 4100
        rawLogFid = -1        % fid for raw line capture; -1 = off
    end
    properties (SetAccess = private)
        nLines = 0            % total lines received
        nSweeps = 0           % of which parsed as RTLS sweeps
        srcIp                 % containers.Map: tag_id (double) -> sender IP (char)
    end
    properties (Access = private)
        sock = []             % java.net.DatagramSocket
        rpkt = []             % reusable receive DatagramPacket
        rbuf = []             % receive byte buffer (int8)
        sweepQ = {}
        eventQ = {}
    end

    methods
        function obj = TagUdp(port)
            if nargin >= 1 && ~isempty(port)
                obj.port = double(port);
            end
            obj.srcIp = containers.Map('KeyType', 'double', 'ValueType', 'char');
        end

        function start(obj)
            % Unbound -> reuse-address -> bind, so re-runs after a crash and a
            % second listener (udp_probe / sniffer) both work. 1 ms SO_TIMEOUT
            % makes receive() effectively non-blocking for the pump loop.
            s = java.net.DatagramSocket([]);
            s.setReuseAddress(true);
            s.setSoTimeout(1);
            s.bind(java.net.InetSocketAddress(int32(obj.port)));
            obj.sock = s;
            obj.rbuf = int8(zeros(1, 2048));
            obj.rpkt = java.net.DatagramPacket(obj.rbuf, numel(obj.rbuf));
        end

        function stop(obj)
            if ~isempty(obj.sock)
                try, obj.sock.close(); catch, end
                obj.sock = [];
            end
        end

        function sendCmd(obj, cmd, tagId)
            %SENDCMD Unicast a command line to a tag by id (host->tag over UDP).
            %   The tag's IP is learned from its stream; it must have been heard
            %   from at least once. Firmware strips the trailing newline.
            if isempty(obj.sock)
                error("dune:TagUdp:notStarted", "Call start() first.");
            end
            if ~obj.srcIp.isKey(double(tagId))
                error("dune:TagUdp:unknownTag", ...
                      "No packet heard from tag %d yet — cannot address it.", tagId);
            end
            ip = obj.srcIp(double(tagId));
            b   = int8(double(char(string(cmd) + newline)));
            dst = java.net.InetAddress.getByName(ip);
            pkt = java.net.DatagramPacket(b, numel(b), dst, int32(obj.port));
            obj.sock.send(pkt);
        end

        function s = drain(obj)
            obj.pump();
            s = obj.sweepQ;
            obj.sweepQ = {};
        end

        function e = drainEvents(obj)
            obj.pump();
            e = obj.eventQ;
            obj.eventQ = {};
        end

        function delete(obj)
            obj.stop();
            if obj.rawLogFid > 0
                fclose(obj.rawLogFid);
                obj.rawLogFid = -1;
            end
        end
    end

    methods (Access = private)
        function pump(obj)
            % Drain every datagram currently queued on the socket. receive()
            % throws SocketTimeoutException once the queue is empty (1 ms
            % SO_TIMEOUT) — that is the loop's normal exit.
            if isempty(obj.sock), return; end
            for iter = 1:5000                     % hard cap: never spin forever
                try
                    obj.rpkt.setLength(numel(obj.rbuf));
                    obj.sock.receive(obj.rpkt);
                catch
                    return;                        % timeout / closed: done
                end
                len = obj.rpkt.getLength();
                if len <= 0, continue; end
                data   = obj.rpkt.getData();
                raw    = char(double(data(1:len)).');
                sender = char(obj.rpkt.getAddress().getHostAddress());
                thost  = posixtime(datetime("now", "TimeZone", "UTC"));
                % One datagram normally carries one line; split defensively.
                for line = split(string(raw), newline).'
                    obj.ingestLine(line, thost, sender);
                end
            end
        end

        function ingestLine(obj, line, thost, sender)
            ln = strtrim(line);
            if strlength(ln) == 0, return; end
            obj.nLines = obj.nLines + 1;
            if obj.rawLogFid > 0
                fprintf(obj.rawLogFid, "%.3f\t%s\t%s\n", thost, sender, ln);
            end
            s = dune.parseRtlsLine(ln);
            if ~isempty(s)
                s.thost = thost;
                s.srcIp = sender;
                obj.srcIp(s.tag) = sender;          % learn tag_id -> IP
                obj.sweepQ{end+1} = s;
                obj.nSweeps = obj.nSweeps + 1;
            else
                obj.eventQ{end+1} = struct('thost', thost, ...
                                           'srcIp', sender, 'line', char(ln));
            end
        end
    end
end
