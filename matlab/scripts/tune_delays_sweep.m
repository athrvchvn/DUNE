function report = tune_delays_sweep(port, opts)
%TUNE_DELAYS_SWEEP Sweep-referenced antenna-delay tuning (feedback loop).
%   report = tune_delays_sweep("COM12")
%
%   Why this exists (2026-07-20 finding): HWCALIB's rapid burst ranging and
%   the production ring-sweep ranging disagree by a CONSTANT per-anchor
%   offset (+60..+500 mm), even though both call the same rangeTo(). So
%   calibrating on the burst path leaves the sweep path - the one the live
%   pipeline uses - biased. This tool closes the loop on the SWEEP path:
%
%     measure sweep medians vs Kinect truth (tag_truth.json)
%       -> per-anchor tick correction (4.69 mm/tick)
%       -> SETANTDELAY push (NVS-persisted)
%       -> re-measure, repeat until all |err| <= tol_mm (or maxIter).
%
%   Prereqs: tag parked EXACTLY where clickTagTruth clicked it, all anchors
%   on, live_tag closed. Current per-anchor ticks are read from
%   matlab/config/delay_calibration.json (written by calibrate_delays) or
%   passed via opts.ticks0.

arguments
    port string = ""
    opts.tagId (1,1) double = 240
    opts.truthFile string = ""
    opts.ticks0 double = []            % [1xM] current ticks, order = opts.ids
    opts.ids (1,:) double = []
    opts.measureS (1,1) double = 10
    opts.tol_mm (1,1) double = 20
    opts.maxIter (1,1) double = 4
    opts.mmPerTick (1,1) double = 4.69
    opts.ackTimeout (1,1) double = 15
end

%% Truth + current ticks
if opts.truthFile == ""
    opts.truthFile = fullfile(dune.rootDir(), 'config', 'tag_truth.json');
end
truth = jsondecode(fileread(opts.truthFile));
tags = truth.tags;
if iscell(tags), tags = [tags{:}]; end
kT = find(arrayfun(@(t) t.id == opts.tagId, tags), 1);
assert(~isempty(kT), 'Tag %d not in %s', opts.tagId, opts.truthFile);
tagTruth = tags(kT);

A = dune.loadAnchors();
if isempty(opts.ids), opts.ids = A.ids(:)'; end
M = numel(opts.ids);
trueR = arrayfun(@(id) tagTruth.true_ranges_m.(sprintf('a%d', id)), opts.ids);

ticks = opts.ticks0;
if isempty(ticks)
    calFile = fullfile(dune.rootDir(), 'config', 'delay_calibration.json');
    cal = jsondecode(fileread(calFile));
    ticks = nan(1, M);
    for k = 1:M
        j = find([cal.anchors.id] == opts.ids(k), 1);
        assert(~isempty(j) && isfinite(cal.anchors(j).delay_ticks), ...
               'No known ticks for A%d - pass opts.ticks0', opts.ids(k));
        ticks(k) = cal.anchors(j).delay_ticks;
    end
    fprintf('Current ticks from %s\n', calFile);
end
assert(numel(ticks) == M, 'ticks0 must match ids');

fprintf('=== Sweep-path delay tuning (tag %d reference, tol %.0f mm) ===\n', ...
        opts.tagId, opts.tol_mm);
for k = 1:M
    fprintf('  A%d  true %.3f m  starting ticks %d\n', opts.ids(k), trueR(k), ticks(k));
end

%% Serial
ts = dune.TagSerial(port);
cleanup = onCleanup(@() delete(ts));
rawDir = fullfile(dune.rootDir(), 'logs');
if ~exist(rawDir, 'dir'), mkdir(rawDir); end
ts.rawLogFid = fopen(fullfile(rawDir, ...
    ['tune_raw_' datestr(now, 'yyyymmdd_HHMMSS') '.log']), 'w'); %#ok<TNOW1,DATST>
ts.start();
fprintf('\nConnected to %s. Tag rebooting...\n', ts.port);
pause(3);
flushIO(ts);

%% Feedback loop
hist = struct('iter', {}, 'err_mm', {}, 'ticks', {});
for it = 1:opts.maxIter
    fprintf('\n-- iteration %d: measuring sweep ranges for %.0f s --\n', it, opts.measureS);
    [med, n] = measureRanges(ts, opts.ids, opts.tagId, opts.measureS);
    err = 1000 * (med - trueR);
    fprintf('  id   true m   sweep m    err mm   n     ticks\n');
    for k = 1:M
        fprintf('  A%d   %6.3f   %7.3f   %+7.0f   %3d   %d\n', ...
                opts.ids(k), trueR(k), med(k), err(k), n(k), ticks(k));
    end
    hist(end+1) = struct('iter', it, 'err_mm', round(err), 'ticks', ticks); %#ok<AGROW>

    if all(isfinite(err)) && all(abs(err) <= opts.tol_mm)
        fprintf('  all anchors within %.0f mm - done.\n', opts.tol_mm);
        break;
    end
    if it == opts.maxIter
        fprintf('  max iterations reached.\n');
        break;
    end

    for k = 1:M
        if ~isfinite(err(k)) || abs(err(k)) <= opts.tol_mm, continue; end
        newTicks = round(ticks(k) + err(k) / opts.mmPerTick);
        newTicks = min(max(newTicks, 15800), 16900);
        fprintf('  A%d: %+.0f mm -> ticks %d -> %d\n', opts.ids(k), err(k), ...
                ticks(k), newTicks);
        okPush = false;
        for attempt = 1:2
            ts.send(sprintf('SETANTDELAY,%d,%d', opts.ids(k), newTicks));
            okPush = waitForAck(ts, sprintf('ANTDELAY_ACK,%d,', opts.ids(k)), ...
                                sprintf('ANTDELAY_FAIL,%d,', opts.ids(k)), ...
                                opts.ackTimeout);
            if okPush, break; end
        end
        if okPush
            ticks(k) = newTicks;
        else
            fprintf('  A%d: push FAILED - ticks unchanged\n', opts.ids(k));
        end
    end
end

%% Final proof
err = 1000 * (med - trueR);
fprintf('\n=== SWEEP-PATH PROOF (truth = Kinect clicks, reg RMSE %.1f mm) ===\n', ...
        1000 * truth.registration_rmse_m);
verdicts = strings(1, M);
for k = 1:M
    if abs(err(k)) <= 30,     verdicts(k) = "PASS";
    elseif abs(err(k)) <= 60, verdicts(k) = "OK";
    else,                     verdicts(k) = "CHECK";
    end
    fprintf('  A%d   err %+5.0f mm   ticks %d   %s\n', opts.ids(k), err(k), ...
            ticks(k), verdicts(k));
end
r = nan(numel(A.ids), 1);
for k = 1:M
    c = find(A.ids == opts.ids(k), 1);
    if ~isempty(c), r(c) = med(k); end
end
p = dune.multilaterate(A.pos, r, tagZ=truth.tag_z, gateK=0);
posErr = 1000 * norm(p - [tagTruth.x, tagTruth.y]);
fprintf('  position (%.3f, %.3f) vs clicked truth (%.3f, %.3f): 2D error %.0f mm\n', ...
        p(1), p(2), tagTruth.x, tagTruth.y, posErr);

report = struct( ...
    'schema', 'dune_delay_tuning_sweep_v1', ...
    'timestamp', datestr(now, 'yyyy-mm-ddTHH:MM:SS'), ... %#ok<TNOW1,DATST>
    'method', 'sweep-path feedback tuning via SETANTDELAY vs Kinect click-truth', ...
    'reference_tag', opts.tagId, ...
    'truth_file', char(opts.truthFile), ...
    'truth_tag_xy', [tagTruth.x, tagTruth.y], ...
    'position_err_mm', round(posErr, 1), ...
    'anchors', struct('id', num2cell(opts.ids), ...
                      'true_m', num2cell(round(trueR, 4)), ...
                      'final_med_m', num2cell(round(med, 4)), ...
                      'final_err_mm', num2cell(round(err)), ...
                      'final_ticks', num2cell(ticks), ...
                      'verdict', cellstr(verdicts)), ...
    'history', hist);
outFile = fullfile(dune.rootDir(), 'config', 'delay_tuning_sweep.json');
fh = fopen(outFile, 'w');
fwrite(fh, jsonencode(report, 'PrettyPrint', true));
fclose(fh);
fprintf('\nReport saved: %s\n', outFile);
end

%% ── helpers ────────────────────────────────────────────────────────────────
function flushIO(ts)
for e = ts.drainEvents()
    fprintf('   [dev] %s\n', e{1}.line);
end
ts.drain();
end

function [med, n] = measureRanges(ts, ids, tagId, durS)
SENTINEL = -2147483648;
acc = cell(1, numel(ids));
t0 = tic;
while toc(t0) < durS
    for c = ts.drain()
        s = c{1};
        if s.tag ~= tagId, continue; end
        for k = 1:numel(s.ids)
            j = find(ids == s.ids(k), 1);
            if isempty(j), continue; end
            if s.rx(k) == SENTINEL || s.fp(k) == SENTINEL || s.dist(k) <= 0
                continue;
            end
            acc{j}(end+1) = s.dist(k); %#ok<AGROW>
        end
    end
    ts.drainEvents();
    pause(0.1);
end
med = nan(1, numel(ids));
n = zeros(1, numel(ids));
for j = 1:numel(ids)
    n(j) = numel(acc{j});
    if n(j) > 0, med(j) = median(acc{j}); end
end
end

function ok = waitForAck(ts, ackPrefix, failPrefix, timeout)
ok = false;
t0 = tic;
while toc(t0) < timeout
    for e = ts.drainEvents()
        ln = e{1}.line;
        if startsWith(ln, ackPrefix), ok = true; return; end
        if startsWith(ln, failPrefix), return; end
    end
    ts.drain();
    pause(0.1);
end
end
