function outFile = survey_capture(opts)
%SURVEY_CAPTURE Trigger the anchor self-survey and save the pairwise ranges.
%   outFile = survey_capture
%   outFile = survey_capture(transport="serial", com="COM12")
%   outFile = survey_capture(transport="udp", port=4100)
%
%   Sends the SURVEY command to the tag and captures the anchor-to-anchor range
%   lines it streams back. The tag (whichever one receives the command) tells
%   each anchor pair to range to each other via DS-TWR and reports the averaged
%   distance - the tag's own antenna delay never enters, so the result is a
%   clean measurement of the anchor constellation (see survey_reconstruct).
%
%   Firmware output (HostLink / runSurvey):
%     SURVEY_BEGIN,v1,<pairs>
%     SURVEY,v1,<a>,<b>,<dist_mm>,<ok>      x pairs   (100 samples each, averaged)
%     SURVEY_DONE,v1
%
%   Writes results/survey_runs/survey_<stamp>.txt (the exact SURVEY lines that
%   survey_fit and survey_reconstruct parse). Transport mirrors rover_teleop_uwb:
%   "serial" talks to the wired master over USB; "udp" listens over WiFi.
%
%   NOTE: in the wired build the SURVEY blocks the master's TagLink loop, so the
%   slave gets no GO and keeps its radio idle - exactly one tag orchestrates the
%   air while the surveyed anchors respond. The slave rejoins afterwards.

arguments
    opts.transport (1,1) string = "serial"   % "serial" | "udp"
    opts.com (1,1) string = "COM12"          % master tag COM ("serial")
    opts.port (1,1) double = 4100            % UDP port ("udp")
    opts.timeout (1,1) double = 120          % s hard cap (10 pairs x ~5 s = ~50 s)
    opts.stall (1,1) double = 20             % s with no new SURVEY line -> give up
end

% --- open the link ----------------------------------------------------------
switch lower(opts.transport)
    case "serial"
        tu = dune.TagSerial(opts.com);
        src = sprintf('SERIAL:%s', opts.com);
    case "udp"
        tu = dune.TagUdp(opts.port);
        src = sprintf('UDP:%d', opts.port);
    otherwise
        error('survey_capture:badTransport', ...
              'transport must be "serial" or "udp", got "%s"', opts.transport);
end
cleanup = onCleanup(@() tu.stop());
tu.start();
pause(1.0);

% UDP unicast needs a tag whose IP has been learned from its stream; serial
% just writes to the port. Learn one before flushing in the UDP case.
udpTag = 0;
if lower(opts.transport) == "udp"
    fprintf('survey_capture: %s | listening to learn a tag IP ...\n', src);
    tLearn = tic;
    while toc(tLearn) < 5 && udpTag == 0
        sw = tu.drain();
        for k = 1:numel(sw)
            if isfield(sw{k}, 'tag') && ~isnan(sw{k}.tag), udpTag = sw{k}.tag; break; end
        end
        pause(0.1);
    end
    if udpTag == 0
        error('survey_capture:noTag', ...
              'No tag heard on %s in 5 s - cannot address the SURVEY command.', src);
    end
    fprintf('  addressing tag %d\n', udpTag);
end

tu.drain(); tu.drainEvents();          % flush anything stale
fprintf('survey_capture: %s | triggering SURVEY ...\n', src);

% --- trigger -----------------------------------------------------------------
sendSurvey(tu, opts.transport, udpTag);

% --- collect until SURVEY_DONE ----------------------------------------------
% Each pair is 100 samples (~5 s), so a full 5-anchor survey runs ~50 s. End on
% SURVEY_DONE; the hard cap is only a backstop, and the stall guard catches a
% genuine hang (no SURVEY line for `stall` s) without cutting a slow-but-alive
% survey short - the bug that made an over-tight timeout look like a crash.
lines = strings(0,1);
pairs = zeros(0,4);       % [a b dist_mm ok]
began = false; done = false;
t0 = tic; lastLine = tic;
fprintf('\n  pair    dist_m   ok        (each pair ~5 s; full survey ~50 s)\n');
while toc(t0) < opts.timeout && ~done
    ev = tu.drainEvents();
    for k = 1:numel(ev)
        ln = string(ev{k}.line);
        if startsWith(ln, "SURVEY_BEGIN")
            began = true; lastLine = tic; lines(end+1,1) = ln; %#ok<AGROW>
        elseif startsWith(ln, "SURVEY_DONE")
            lines(end+1,1) = ln; %#ok<AGROW>
            done = true;
        elseif startsWith(ln, "SURVEY,v1,")
            lastLine = tic;
            lines(end+1,1) = ln; %#ok<AGROW>
            t = split(ln, ',');
            if numel(t) >= 6
                a = double(t(3)); b = double(t(4));
                mm = double(t(5)); ok = double(t(6));
                pairs(end+1,:) = [a b mm ok]; %#ok<AGROW>
                fprintf('  A%d-A%d   %6.3f   %3d\n', a, b, mm/1000, ok);
            end
        end
    end
    if began && toc(lastLine) > opts.stall
        warning(['No SURVEY line for %.0f s - survey appears stalled (a pair ' ...
                 'that gets no anchor-to-anchor link takes the full timeout).'], opts.stall);
        break;
    end
    pause(0.05);
end

if ~began
    warning(['No SURVEY_BEGIN heard in %.0f s. Is the master on %s and its ' ...
             'Serial Monitor closed? (Serial mode talks to whichever tag the ' ...
             'command reaches.)'], opts.timeout, src);
end
if ~done
    warning('SURVEY_DONE not received - capture may be incomplete (%d pairs).', ...
            size(pairs,1));
end

% --- save --------------------------------------------------------------------
outDir = fullfile(dune.rootDir(), 'results', 'survey_runs');
if ~exist(outDir, 'dir'), mkdir(outDir); end
stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
outFile = fullfile(outDir, sprintf('survey_%s.txt', stamp));
fid = fopen(outFile, 'w');
for i = 1:numel(lines), fprintf(fid, '%s\n', lines(i)); end
fclose(fid);

% Expected pair count tracks whatever the FIRMWARE currently surveys, which is
% not necessarily numel(anchors.json) - they can briefly disagree mid-procedure
% (e.g. anchors.json already trimmed before a reflash). Report both counts
% seen among the pairs actually received, which is unambiguous either way.
fprintf('\nCaptured %d pairs to %s\n', size(pairs,1), outFile);
if isempty(pairs)
    fprintf('No pairs captured at all - check the link (see warnings above).\n');
else
    nAnchSeen = numel(unique(pairs(:,1:2)));
    nPairsExpected = nAnchSeen * (nAnchSeen - 1) / 2;
    if size(pairs,1) < nPairsExpected
        fprintf(['Expected %d pairs for the %d anchors seen - a missing pair usually ' ...
                 'means a weak/off anchor.\n'], nPairsExpected, nAnchSeen);
    end
end
fprintf('Next: survey_reconstruct("%s")\n', outFile);
end

% ---------------------------------------------------------------------------
function sendSurvey(tu, transport, udpTag)
% TagSerial exposes send(cmd); TagUdp exposes sendCmd(cmd, tagId).
if lower(transport) == "udp"
    tu.sendCmd("SURVEY", udpTag);   % unicast to the learned tag's IP
else
    tu.send("SURVEY");
end
end
