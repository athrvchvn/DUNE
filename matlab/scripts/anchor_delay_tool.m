function results = anchor_delay_tool(port, opts)
%ANCHOR_DELAY_TOOL Push antenna-delay values into the anchors' NVS over UWB.
%   anchor_delay_tool                    % factory 16434 -> anchors 1..5
%   anchor_delay_tool("COM7", ids=[3 5]) % only these anchors
%   anchor_delay_tool("", ticks=16450)   % custom value
%
%   Connects to the tag over COM and, for each anchor id, sends
%   SETANTDELAY,<id>,<ticks>; the tag relays it over UWB and the anchor
%   stores it in NVS (survives reboot AND reflash). Waits for the
%   ANTDELAY_ACK / ANTDELAY_FAIL response, retries once on failure.
%   Anchors must be POWERED ON to acknowledge.
%
%   Also queries the tag's own delay (GETMYDELAY). To change the tag's own
%   delay: opts.tagTicks (sent as SETMYDELAY, NVS-persisted on the tag).
%
%   Factory default 16434 is the DW1000 nominal - use it to wipe suspect
%   calibration junk from NVS; proper per-board tuning is step 4 (HWCALIB
%   against Kinect click-truth distances).
%
%   Returns a struct array: id, ticks, ok.

arguments
    port string = ""
    opts.ids (1,:) double = [1 2 3 4 5]
    opts.ticks (1,1) double = 16434
    opts.tagId (1,1) double = 240
    opts.tagTicks double = []          % [] = only query, don't change
    opts.timeout (1,1) double = 10     % s to wait for each ACK
end

ts = dune.TagSerial(port);
cleanup = onCleanup(@() delete(ts));
ts.start();
fprintf('Connected to %s. Waiting for tag boot/stream...\n', ts.port);
pause(3);                              % opening the port resets the tag
drainPrint(ts);

results = struct('id', {}, 'ticks', {}, 'ok', {});
for id = opts.ids
    ok = false;
    for attempt = 1:2
        fprintf('>> SETANTDELAY,%d,%d (attempt %d)\n', id, opts.ticks, attempt);
        ts.send(sprintf('SETANTDELAY,%d,%d', id, opts.ticks));
        ok = waitForAck(ts, sprintf('ANTDELAY_ACK,%d,', id), ...
                        sprintf('ANTDELAY_FAIL,%d,', id), opts.timeout);
        if ok, break; end
    end
    if ok
        fprintf('   A%d OK - delay %d saved to its NVS\n', id, opts.ticks);
    else
        fprintf('   A%d FAILED - is it powered on and in range?\n', id);
    end
    results(end+1) = struct('id', id, 'ticks', opts.ticks, 'ok', ok); %#ok<AGROW>
end

%% Tag's own delay
if ~isempty(opts.tagTicks)
    fprintf('>> SETMYDELAY,%d,%d\n', opts.tagId, opts.tagTicks);
    ts.send(sprintf('SETMYDELAY,%d,%d', opts.tagId, opts.tagTicks));
    waitForAck(ts, 'MYDELAY_ACK,', 'MYDELAY_FAIL,', opts.timeout);
else
    ts.send(sprintf('GETMYDELAY,%d', opts.tagId));
    waitForAck(ts, 'MYDELAY_ACK,', 'MYDELAY_FAIL,', opts.timeout);
end

nOk = nnz([results.ok]);
fprintf('\nDone: %d/%d anchors updated.\n', nOk, numel(opts.ids));
if nOk < numel(opts.ids)
    fprintf('Re-run for the failed ones, e.g. anchor_delay_tool(ids=[%s])\n', ...
            num2str([results(~[results.ok]).id]));
end
end

function ok = waitForAck(ts, ackPrefix, failPrefix, timeout)
% Poll device lines until the ACK/FAIL for this command shows up.
ok = false;
tEnd = tic;
while toc(tEnd) < timeout
    for e = ts.drainEvents()
        ln = e{1}.line;
        fprintf('   [dev] %s\n', ln);
        if startsWith(ln, ackPrefix), ok = true; return; end
        if startsWith(ln, failPrefix), return; end
    end
    ts.drain();                        % discard RTLS sweeps meanwhile
    pause(0.1);
end
fprintf('   (no response within %.0f s)\n', timeout);
end

function drainPrint(ts)
for e = ts.drainEvents()
    fprintf('   [dev] %s\n', e{1}.line);
end
ts.drain();
end
