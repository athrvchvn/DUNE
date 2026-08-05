function report = tune_tag_delay(port, opts)
%TUNE_TAG_DELAY Tune a tag's OWN antenna delay on the sweep path (step 7.1).
%   report = tune_tag_delay("COM7")          % tag 241 by default
%
%   Counterpart of tune_delays_sweep, but adjusts the TAG's delay
%   (SETMYDELAY) and NEVER touches the anchors: the anchors were tuned with
%   tag 240 as the reference, so a second tag must absorb its own offset.
%   A tag delay error shifts EVERY range by the same constant, so the loop
%   corrects the MEDIAN-ACROSS-ANCHORS error each iteration.
%
%   Prereqs:
%     1. clickTagTruth(0.24, <tagId>) with the unit parked (writes truth).
%     2. THIS tag on USB (its own COM port), all anchors on, unit unmoved.
%     3. The DW1000 power correction is applied during measurement
%        (production-consistent), so anchors' residuals stay out of the fit.

arguments
    port string = ""
    opts.tagId (1,1) double = 241
    opts.truthFile string = ""
    opts.tagZ (1,1) double = 0.24
    opts.measureS (1,1) double = 10
    opts.tol_mm (1,1) double = 15
    opts.maxIter (1,1) double = 4
    opts.mmPerTick (1,1) double = 4.69
    opts.ackTimeout (1,1) double = 10
end

%% Truth
if opts.truthFile == ""
    opts.truthFile = fullfile(dune.rootDir(), 'config', 'tag_truth.json');
end
truth = jsondecode(fileread(opts.truthFile));
tags = truth.tags;
if iscell(tags), tags = [tags{:}]; end
kT = find(arrayfun(@(t) t.id == opts.tagId, tags), 1);
assert(~isempty(kT), 'Tag %d not in %s - run clickTagTruth(0.24, %d) first.', ...
       opts.tagId, opts.truthFile, opts.tagId);
tagTruth = tags(kT);

A = dune.loadAnchors();
RC = dune.loadRangeCorrection();
M = numel(A.ids);
trueR = arrayfun(@(id) tagTruth.true_ranges_m.(sprintf('a%d', id)), A.ids);

fprintf('=== Tag %d own-delay tuning (anchors untouched) ===\n', opts.tagId);
fprintf('Truth: tag at (%.3f, %.3f), power corr %s\n', tagTruth.x, tagTruth.y, ...
        string(~isempty(RC)));

%% Serial
ts = dune.TagSerial(port);
cleanup = onCleanup(@() delete(ts));
rawDir = fullfile(dune.rootDir(), 'logs');
if ~exist(rawDir, 'dir'), mkdir(rawDir); end
ts.rawLogFid = fopen(fullfile(rawDir, sprintf('tunetag%d_raw_%s.log', opts.tagId, ...
    datestr(now, 'yyyymmdd_HHMMSS'))), 'w'); %#ok<TNOW1,DATST>
ts.start();
fprintf('Connected to %s. Tag rebooting...\n', ts.port);
pause(3);
for e = ts.drainEvents(), fprintf('   [dev] %s\n', e{1}.line); end
ts.drain();

ts.send(sprintf('GETMYDELAY,%d', opts.tagId));
ticks = waitDelayAck(ts, opts.ackTimeout);
assert(isfinite(ticks), 'No MYDELAY_ACK - is tag %d on this port?', opts.tagId);
fprintf('Current own delay: %d ticks\n', ticks);

%% Feedback loop on the median-across-anchors error
hist = struct('iter', {}, 'commonErrMm', {}, 'ticks', {});
for it = 1:opts.maxIter
    fprintf('\n-- iteration %d: measuring %d s of sweeps --\n', it, opts.measureS);
    [med, n, pos] = measureCorrected(ts, A, RC, opts.tagId, opts.tagZ, opts.measureS);
    err = 1000 * (med - trueR);
    fprintf('  id   true m   meas m    err mm   n\n');
    for c = 1:M
        fprintf('  A%d   %6.3f   %6.3f   %+7.0f   %3d\n', A.ids(c), trueR(c), ...
                med(c), err(c), n(c));
    end
    common = median(err, 'omitnan');
    fprintf('  common offset (median across anchors): %+.0f mm   position (%.3f, %.3f)\n', ...
            common, pos(1), pos(2));
    hist(end+1) = struct('iter', it, 'commonErrMm', round(common), 'ticks', ticks); %#ok<AGROW>

    if abs(common) <= opts.tol_mm
        fprintf('  within %.0f mm - done.\n', opts.tol_mm);
        break;
    end
    if it == opts.maxIter, fprintf('  max iterations reached.\n'); break; end

    newTicks = min(max(round(ticks + common / opts.mmPerTick), 15800), 16900);
    fprintf('  SETMYDELAY: ticks %d -> %d\n', ticks, newTicks);
    ts.send(sprintf('SETMYDELAY,%d,%d', opts.tagId, newTicks));
    got = waitDelayAck(ts, opts.ackTimeout);
    if isfinite(got)
        ticks = got;
    else
        fprintf('  no ACK - ticks unchanged\n');
    end
end

%% Proof
err = 1000 * (med - trueR);
posErr = 1000 * norm(pos - [tagTruth.x, tagTruth.y]);
fprintf('\n=== TAG %d PROOF ===\n', opts.tagId);
for c = 1:M
    fprintf('  A%d  residual %+5.0f mm\n', A.ids(c), err(c));
end
fprintf('  final own delay %d ticks, common offset %+.0f mm, 2D position error %.0f mm\n', ...
        ticks, median(err, 'omitnan'), posErr);

report = struct('schema', 'dune_tag_delay_tuning_v1', ...
    'timestamp', datestr(now, 'yyyy-mm-ddTHH:MM:SS'), ... %#ok<TNOW1,DATST>
    'tag', opts.tagId, 'final_ticks', ticks, ...
    'residual_err_mm', round(err(:))', ...
    'position_err_mm', round(posErr, 1), ...
    'truth_xy', [tagTruth.x, tagTruth.y], 'history', hist);
outFile = fullfile(dune.rootDir(), 'config', ...
                   sprintf('delay_tuning_tag%d.json', opts.tagId));
fh = fopen(outFile, 'w');
fwrite(fh, jsonencode(report, 'PrettyPrint', true));
fclose(fh);
fprintf('Report saved: %s\n', outFile);
end

%% helpers
function ticks = waitDelayAck(ts, timeout)
ticks = NaN;
t0 = tic;
while toc(t0) < timeout
    for e = ts.drainEvents()
        tok = regexp(e{1}.line, '^MYDELAY_ACK,\d+,(\d+)', 'tokens', 'once');
        if ~isempty(tok), ticks = str2double(tok{1}); return; end
    end
    ts.drain();
    pause(0.1);
end
end

function [med, n, pos] = measureCorrected(ts, A, RC, tagId, tagZ, durS)
% Median POWER-CORRECTED range per anchor + median position over durS.
M = numel(A.ids);
acc = cell(1, M);
Ps = [];
prev = [];
t0 = tic;
while toc(t0) < durS
    for c = ts.drain()
        s = c{1};
        if s.tag ~= tagId, continue; end
        [p, info] = dune.solveSweep(s, A, rangeCorr=RC, tagZ=tagZ, x0=prev);
        if all(isfinite(p)), Ps(end+1, :) = p; prev = p; end %#ok<AGROW>
        for k = 1:M
            if isfinite(info.rangeCorr(k))
                acc{k}(end+1) = info.rangeCorr(k); %#ok<AGROW>
            end
        end
    end
    ts.drainEvents();
    pause(0.1);
end
med = nan(M, 1); n = zeros(M, 1);
for k = 1:M
    n(k) = numel(acc{k});
    if n(k) > 0, med(k) = median(acc{k}); end
end
pos = [NaN, NaN];
if ~isempty(Ps), pos = median(Ps, 1); end
end
