function report = calibrate_delays(port, opts)
%CALIBRATE_DELAYS Step-4 on-device antenna-delay calibration, with proof.
%   report = calibrate_delays("COM12")
%   calibrate_delays("COM12", dryRun=true)   % check the plan, no serial
%
%   Prerequisites (in this order):
%     1. Tag parked with line of sight to all anchors, and NOT moved since
%        clickTagTruth wrote matlab/config/tag_truth.json (Kinect truth).
%     2. All anchors powered. live_tag closed (frees the COM port).
%
%   What it does, per anchor:
%     sends HWCALIB,<id>,<true_mm> -> the tag binary-searches that anchor's
%     antenna delay ON-DEVICE (14 iterations x 50 ranges, window
%     15800..16900 ticks ~= +/-2.6 m), pushing each candidate over UWB,
%     until measured distance matches the Kinect truth. The anchor persists
%     the final delay in NVS (survives reboot and reflash).
%
%   Proof of calibration:
%     raw ranges are measured for opts.measureS seconds BEFORE and AFTER;
%     the report prints per-anchor range error vs truth and the 2D position
%     error of a plain solve on the median ranges, and everything is saved
%     to matlab/config/delay_calibration.json (provenance for the paper).
%
%   NOTE tag 240 is the REFERENCE: its own delay error is absorbed into the
%   anchor values (pair ranging can't separate them). Tag 241 is aligned to
%   the same reference later via HWCALIB_SELF against a calibrated anchor.

arguments
    port string = ""
    opts.tagId (1,1) double = 240
    opts.truthFile string = ""
    opts.ids (1,:) double = []          % default: all anchors in anchors.json
    opts.measureS (1,1) double = 12
    opts.ackTimeout (1,1) double = 180  % s per anchor (search takes ~20-60 s)
    opts.maxAge_min (1,1) double = 45
    opts.dryRun (1,1) logical = false
    opts.verifyOnly (1,1) logical = false   % just re-measure + report, no HWCALIB
end

%% Truth
if opts.truthFile == ""
    opts.truthFile = fullfile(dune.rootDir(), 'config', 'tag_truth.json');
end
truth = jsondecode(fileread(opts.truthFile));
tags = truth.tags;
if iscell(tags), tags = [tags{:}]; end
kT = find(arrayfun(@(t) t.id == opts.tagId, tags), 1);
if isempty(kT)
    error('calibrate_delays:noTag', 'Tag %d not in %s - re-run clickTagTruth.', ...
          opts.tagId, opts.truthFile);
end
tagTruth = tags(kT);
try
    age = minutes(datetime('now') - datetime(tagTruth0Time(truth), ...
          'InputFormat', 'yyyy-MM-dd''T''HH:mm:ss'));
catch
    age = NaN;
end
if isfinite(age) && age > opts.maxAge_min
    warning('Truth is %.0f min old - is the tag still exactly where you clicked it?', age);
end

A = dune.loadAnchors();
if isempty(opts.ids), opts.ids = A.ids(:)'; end
M = numel(opts.ids);
trueR = nan(1, M);
for k = 1:M
    f = sprintf('a%d', opts.ids(k));
    if ~isfield(tagTruth.true_ranges_m, f)
        error('calibrate_delays:noRange', 'No truth range for anchor %d.', opts.ids(k));
    end
    trueR(k) = tagTruth.true_ranges_m.(f);
end
if any(trueR < 0.3 | trueR > 20)
    error('calibrate_delays:implausible', 'Truth range outside 0.3-20 m - bad click?');
end

fprintf('=== DUNE delay calibration (tag %d = reference) ===\n', opts.tagId);
fprintf('Truth: %s  (tag at %.3f, %.3f, z %.2f; registration RMSE %.1f mm)\n', ...
        opts.truthFile, tagTruth.x, tagTruth.y, truth.tag_z, ...
        1000 * truth.registration_rmse_m);
for k = 1:M
    fprintf('  A%d  true %.3f m  -> HWCALIB,%d,%d\n', opts.ids(k), trueR(k), ...
            opts.ids(k), round(1000 * trueR(k)));
end
if opts.dryRun
    fprintf('Dry run - no serial traffic.\n');
    report = [];
    return;
end

%% Serial
ts = dune.TagSerial(port);
cleanup = onCleanup(@() delete(ts));
rawDir = fullfile(dune.rootDir(), 'logs');
if ~exist(rawDir, 'dir'), mkdir(rawDir); end
ts.rawLogFid = fopen(fullfile(rawDir, ...
    ['calib_raw_' datestr(now, 'yyyymmdd_HHMMSS') '.log']), 'w'); %#ok<TNOW1,DATST>
ts.start();
fprintf('\nConnected to %s. Tag rebooting...\n', ts.port);
pause(3);
printEvents(ts);
ts.send(sprintf('GETMYDELAY,%d', opts.tagId));
tagTicks = waitTagDelay(ts, 10);
fprintf('Tag %d own delay: %s ticks (unchanged by this procedure)\n', ...
        opts.tagId, string(tagTicks));

%% BEFORE measurement
fprintf('\n-- BEFORE: measuring raw ranges for %.0f s (do not move anything) --\n', ...
        opts.measureS);
[befMed, befN] = measureRanges(ts, opts.ids, opts.tagId, opts.measureS);
printErrTable(opts.ids, trueR, befMed, befN);
[befPos, befErr] = posCheck(A, opts.ids, befMed, truth.tag_z, tagTruth);
fprintf('  position: (%.3f, %.3f) -> error vs clicked truth %.0f mm\n', ...
        befPos(1), befPos(2), befErr);

%% Calibrate each anchor
final = nan(1, M);
ok = false(1, M);
if opts.verifyOnly
    fprintf('\n-- verifyOnly: skipping HWCALIB, re-measuring only --\n');
    ok(:) = true;
end
for k = 1:(M * ~opts.verifyOnly)
    id = opts.ids(k);
    if isnan(befMed(k))
        fprintf('\n== A%d: NO RANGES in the before-measurement - attempting anyway ==\n', id);
    else
        fprintf('\n== A%d: true %.3f m, currently reads %+.0f mm off ==\n', ...
                id, trueR(k), 1000 * (befMed(k) - trueR(k)));
    end
    for attempt = 1:2
        ts.send(sprintf('HWCALIB,%d,%d', id, round(1000 * trueR(k))));
        [ok(k), final(k), why] = waitCalib(ts, id, opts.ackTimeout);
        if ok(k)
            fprintf('  A%d DONE: delay %d saved to its NVS\n', id, final(k));
            break;
        end
        fprintf('  A%d FAILED (%s), attempt %d\n', id, why, attempt);
    end
end

%% AFTER measurement
fprintf('\n-- AFTER: measuring raw ranges for %.0f s --\n', opts.measureS);
[aftMed, aftN] = measureRanges(ts, opts.ids, opts.tagId, opts.measureS);

%% Verdict
fprintf('\n=== CALIBRATION PROOF (truth = Kinect clicks, reg RMSE %.1f mm) ===\n', ...
        1000 * truth.registration_rmse_m);
fprintf('  id   true m   before err   after err    delay   verdict\n');
verdicts = strings(1, M);
for k = 1:M
    be = 1000 * (befMed(k) - trueR(k));
    ae = 1000 * (aftMed(k) - trueR(k));
    if ~ok(k),            v = "FAILED";
    elseif abs(ae) <= 30, v = "PASS";
    elseif abs(ae) <= 60, v = "OK";
    else,                 v = "CHECK";
    end
    verdicts(k) = v;
    fprintf('  A%d   %6.3f   %+7.0f mm   %+7.0f mm   %6s   %s\n', ...
            opts.ids(k), trueR(k), be, ae, num2str(final(k)), v);
end
[aftPos, aftErr] = posCheck(A, opts.ids, aftMed, truth.tag_z, tagTruth);
fprintf('  position after: (%.3f, %.3f)   2D error vs truth: before %.0f mm -> after %.0f mm\n', ...
        aftPos(1), aftPos(2), befErr, aftErr);

%% Report file
report = struct( ...
    'schema', 'dune_delay_calibration_v1', ...
    'timestamp', datestr(now, 'yyyy-mm-ddTHH:MM:SS'), ... %#ok<TNOW1,DATST>
    'method', 'HWCALIB on-device binary search vs Kinect click-truth', ...
    'reference_tag', opts.tagId, ...
    'reference_tag_delay_ticks', tagTicks, ...
    'truth_file', char(opts.truthFile), ...
    'truth_tag_xy', [tagTruth.x, tagTruth.y], ...
    'truth_tag_z', truth.tag_z, ...
    'registration_rmse_m', truth.registration_rmse_m, ...
    'position_err_mm_before', round(befErr, 1), ...
    'position_err_mm_after', round(aftErr, 1), ...
    'anchors', struct('id', num2cell(opts.ids), ...
                      'true_m', num2cell(round(trueR, 4)), ...
                      'before_med_m', num2cell(round(befMed, 4)), ...
                      'after_med_m', num2cell(round(aftMed, 4)), ...
                      'before_err_mm', num2cell(round(1000 * (befMed - trueR))), ...
                      'after_err_mm', num2cell(round(1000 * (aftMed - trueR))), ...
                      'n_before', num2cell(befN), 'n_after', num2cell(aftN), ...
                      'delay_ticks', num2cell(final), ...
                      'ok', num2cell(ok), ...
                      'verdict', cellstr(verdicts)));
outFile = fullfile(dune.rootDir(), 'config', 'delay_calibration.json');
fh = fopen(outFile, 'w');
fwrite(fh, jsonencode(report, 'PrettyPrint', true));
fclose(fh);
fprintf('\nReport saved: %s\n', outFile);
if any(~ok)
    fprintf('Re-run failed anchors with: calibrate_delays("%s", ids=[%s])\n', ...
            ts.port, num2str(opts.ids(~ok)));
end
end

%% ── helpers ────────────────────────────────────────────────────────────────
function tstr = tagTruth0Time(truth)
tstr = '';
if isfield(truth, 'timestamp'), tstr = truth.timestamp; end
end

function printEvents(ts)
for e = ts.drainEvents()
    fprintf('   [dev] %s\n', e{1}.line);
end
ts.drain();
end

function ticks = waitTagDelay(ts, timeout)
ticks = NaN;
t0 = tic;
while toc(t0) < timeout
    for e = ts.drainEvents()
        ln = e{1}.line;
        tok = regexp(ln, '^MYDELAY_ACK,\d+,(\d+)', 'tokens', 'once');
        if ~isempty(tok), ticks = str2double(tok{1}); return; end
    end
    ts.drain();
    pause(0.1);
end
end

function [med, n] = measureRanges(ts, ids, tagId, durS)
% Median raw range per anchor over durS seconds of live sweeps.
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

function printErrTable(ids, trueR, med, n)
fprintf('  id   true m   measured m   err mm   samples\n');
for k = 1:numel(ids)
    fprintf('  A%d   %6.3f   %8.3f   %+7.0f   %5d\n', ids(k), trueR(k), ...
            med(k), 1000 * (med(k) - trueR(k)), n(k));
end
end

function [p, errMm] = posCheck(A, ids, med, tagZ, tagTruth)
% Plain unweighted solve on the median ranges vs the clicked truth position.
r = nan(numel(A.ids), 1);
for k = 1:numel(ids)
    c = find(A.ids == ids(k), 1);
    if ~isempty(c), r(c) = med(k); end
end
p = dune.multilaterate(A.pos, r, tagZ=tagZ, gateK=0);
errMm = 1000 * norm(p - [tagTruth.x, tagTruth.y]);
end

function [ok, delay, why] = waitCalib(ts, id, timeout)
% Stream HWCALIB progress until DONE/FAIL for this anchor.
ok = false; delay = NaN; why = 'timeout';
t0 = tic;
while toc(t0) < timeout
    for e = ts.drainEvents()
        ln = e{1}.line;
        tok = regexp(ln, sprintf('^HWCALIB_PROG,%d,(\\d+),(\\d+),(\\d+),(-?\\d+)', id), ...
                     'tokens', 'once');
        if ~isempty(tok)
            v = str2double(tok);
            fprintf('   it %2d/14: delay %5d -> meas %.3f m (err %+d mm)\n', ...
                    v(1) + 1, v(2), v(3) / 1000, v(4));
            continue;
        end
        tok = regexp(ln, sprintf('^HWCALIB_DONE,%d,(\\d+)', id), 'tokens', 'once');
        if ~isempty(tok)
            ok = true; delay = str2double(tok{1});
            return;
        end
        tok = regexp(ln, sprintf('^HWCALIB_FAIL,%d,(\\S+)', id), 'tokens', 'once');
        if ~isempty(tok)
            why = tok{1};
            return;
        end
        fprintf('   [dev] %s\n', ln);
    end
    ts.drain();
    pause(0.1);
end
end
