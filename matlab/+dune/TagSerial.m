classdef TagSerial < handle
    %TAGSERIAL Line-based COM link to a DUNE tag (protocol-agnostic ingest).
    %   ts = dune.TagSerial()          % auto-detect (needs exactly 1 free port)
    %   ts = dune.TagSerial("COM7")
    %   ts.start()                     % open port, begin async line capture
    %   sweeps = ts.drain()            % parsed sweeps since the last drain
    %   events = ts.drainEvents()      % non-RTLS lines (boot, MYDELAY_ACK...)
    %   ts.send("GETMYDELAY,240")      % calibration/control commands
    %   ts.stop()
    %
    %   Each parsed sweep is the dune.parseRtlsLine struct plus .thost
    %   (posix s, host receive time). Ingest stays protocol-agnostic: the
    %   consumer only ever sees sweep structs, so a future broadcast-POLL
    %   firmware only needs its own parser here, nothing downstream changes.
    %
    %   Set .rawLogFid (an fopen'd fid) BEFORE start() to tee every incoming
    %   line, tab-stamped with t_host, into a raw capture file.

    properties
        port
        baud = 115200
        rawLogFid = -1        % fid for raw line capture; -1 = off
    end
    properties (SetAccess = private)
        sp = []               % serialport object
        nLines = 0            % total lines received
        nSweeps = 0           % of which parsed as RTLS sweeps
    end
    properties (Access = private)
        sweepQ = {}
        eventQ = {}
    end

    methods
        function obj = TagSerial(port, baud)
            if nargin < 1 || strlength(string(port)) == 0
                avail = serialportlist("available");
                if isscalar(avail)
                    port = avail(1);
                elseif isempty(avail)
                    error("dune:TagSerial:noPort", ...
                          "No free serial port found. Is the tag plugged in?");
                else
                    error("dune:TagSerial:ambiguous", ...
                          "Several ports free (%s) - pass the tag's port explicitly.", ...
                          strjoin(avail, ", "));
                end
            end
            obj.port = string(port);
            if nargin >= 2, obj.baud = baud; end
        end

        function start(obj)
            obj.sp = serialport(obj.port, obj.baud);
            configureTerminator(obj.sp, "LF");
            obj.sp.Timeout = 2;
            flush(obj.sp);
            configureCallback(obj.sp, "terminator", @(src, ~) obj.onLine(src));
        end

        function stop(obj)
            if ~isempty(obj.sp)
                configureCallback(obj.sp, "off");
                obj.sp = [];                      % releases the COM port
            end
        end

        function send(obj, cmd)
            writeline(obj.sp, string(cmd));
        end

        function s = drain(obj)
            s = obj.sweepQ;
            obj.sweepQ = {};
        end

        function e = drainEvents(obj)
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
        function onLine(obj, src)
            try
                line = readline(src);
            catch
                return;
            end
            if ismissing(line), return; end
            thost = posixtime(datetime("now", "TimeZone", "UTC"));
            obj.nLines = obj.nLines + 1;
            if obj.rawLogFid > 0
                fprintf(obj.rawLogFid, "%.3f\t%s\n", thost, line);
            end
            s = dune.parseRtlsLine(line);
            if ~isempty(s)
                s.thost = thost;
                obj.sweepQ{end+1} = s;
                obj.nSweeps = obj.nSweeps + 1;
            else
                ln = strtrim(line);
                if strlength(ln) > 0
                    obj.eventQ{end+1} = struct('thost', thost, 'line', char(ln));
                end
            end
        end
    end
end
