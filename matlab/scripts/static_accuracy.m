function out = static_accuracy(port, opts)
%STATIC_ACCURACY Step-5 multi-spot static accuracy vs Kinect click-truth.
%   out = static_accuracy("COM12")
%
%   Per spot: park the tag, press ENTER, click the tag antenna in the fresh
%   Kinect snapshot (= truth), then it records opts.measureS seconds of live
%   sweeps and solves every one through the full live path (sentinel reject
%   -> NLOS gap weights -> weighted LM). Type q at the prompt to finish the
%   session. Do 4-6 spots: centre, near an edge/anchor, corners.
%
%   Reports per spot: truth, median solved position, fixed offset (median -
%   truth), scatter, per-sweep 2D error stats, per-anchor range error
%   medians (seed data for the Tier-2 spatial map). Overall: RMSE over all
%   sweeps vs the <=30 mm goal. Outputs (report.txt, spots.json, map.png)
%   go to matlab\results\static_accuracy_<stamp>\.

arguments
    port string = ""
    opts.tagId (1,1) double = 240
    opts.tagZ (1,1) double = 0.24    % must match how the truth is clicked
    opts.measureS (1,1) double = 20
    opts.powerCorr (1,1) logical = true
end

A = dune.loadAnchors();
RC = [];
if opts.powerCorr, RC = dune.loadRangeCorrection(); end

stamp = datestr(now, 'yyyymmdd_HHMMSS'); %#ok<TNOW1,DATST>
outDir = fullfile(dune.rootDir(), 'results', ['static_accuracy_' stamp]);
mkdir(outDir);
rpt = fopen(fullfile(outDir, 'report.txt'), 'w');
closeRpt = onCleanup(@() fclose(rpt));

ts = dune.TagSerial(port);
cleanup = onCleanup(@() delete(ts));
ts.rawLogFid = fopen(fullfile(outDir, 'serial_raw.log'), 'w');
ts.start();
fprintf('Connected to %s. Tag rebooting...\n', ts.port);
pause(3);
ts.drain(); ts.drainEvents();

pr(rpt, 'DUNE step-5 static accuracy — %s', stamp);
pr(rpt, 'Anchors: %s (%s)', A.file, A.layout);
pr(rpt, 'Tag %d, tagZ %.2f m, %d s per spot, power corr %s', ...
   opts.tagId, opts.tagZ, opts.measureS, string(~isempty(RC)));

spots = struct('k', {}, 'truth', {}, 'medPos', {}, 'biasMm', {}, ...
               'stats', {}, 'anchorErrMm', {}, 'n', {});
allErr = [];
k = 0;
while true
    ans_ = input(sprintf(['\nSpot %d: park the tag, then ENTER to click truth ' ...
                          '(q = finish): '], k + 1), 's');
    if strcmpi(strtrim(ans_), 'q'), break; end
    k = k + 1;

    % Truth click (dryRun: don't overwrite the calibration tag_truth.json)
    truth = clickTagTruth(opts.tagZ, struct('uwbTagIds', opts.tagId, 'dryRun', true));
    tXY = [truth.tags(1).x, truth.tags(1).y];
    trueR = arrayfun(@(id) truth.tags(1).true_ranges_m.(sprintf('a%d', id)), A.ids);

    fprintf('Measuring %d s of sweeps — do not move the tag...\n', opts.measureS);
    ts.drain(); ts.drainEvents();               % flush anything stale
    sweeps = collectSweeps(ts, opts.tagId, opts.measureS);
    if isempty(sweeps)
        pr(rpt, 'Spot %d: NO SWEEPS RECEIVED — check the stream. Spot skipped.', k);
        k = k - 1;
        continue;
    end

    N = numel(sweeps);
    P = nan(N, 2);
    R = nan(N, numel(A.ids));
    prev = [];
    for i = 1:N
        [p, info] = dune.solveSweep(sweeps{i}, A, rangeCorr=RC, ...
                                    tagZ=opts.tagZ, x0=prev);
        P(i, :) = p;
        R(i, :) = info.range';
        if all(isfinite(p)), prev = p; end
    end
    solved = ~isnan(P(:, 1));
    err = vecnorm(P(solved, :) - tXY, 2, 2);
    medPos = median(P(solved, :), 1);
    bias = 1000 * (medPos - tXY);
    st = struct('n', N, 'solvedPct', 100 * mean(solved), ...
                'rmse_mm', 1000 * sqrt(mean(err.^2)), ...
                'median_mm', 1000 * median(err), ...
                'p95_mm', 1000 * prctile(err, 95), ...
                'max_mm', 1000 * max(err), ...
                'scatterStd_mm', 1000 * std(vecnorm(P(solved, :) - medPos, 2, 2)));
    aerr = 1000 * (median(R, 1, 'omitnan')' - trueR);

    pr(rpt, '\nSpot %d: truth (%.3f, %.3f)   sweeps %d (%.0f%% solved)', ...
       k, tXY(1), tXY(2), N, st.solvedPct);
    pr(rpt, '  median pos (%.3f, %.3f)   offset [%+.0f %+.0f] mm (|%.0f| mm)', ...
       medPos(1), medPos(2), bias(1), bias(2), norm(bias));
    pr(rpt, '  2D err: RMSE %.0f | median %.0f | p95 %.0f | max %.0f mm   scatter std %.0f mm', ...
       st.rmse_mm, st.median_mm, st.p95_mm, st.max_mm, st.scatterStd_mm);
    pr(rpt, '  per-anchor range err (med, mm): %s', ...
       strjoin(compose('A%d %+.0f', [double(A.ids), aerr]), '  '));

    spots(end+1) = struct('k', k, 'truth', tXY, 'medPos', medPos, ...
        'biasMm', round(bias, 1), 'stats', st, ...
        'anchorErrMm', round(aerr'), 'n', N); %#ok<AGROW>
    allErr = [allErr; err]; %#ok<AGROW>
end

if isempty(spots)
    pr(rpt, 'No spots recorded.');
    out = [];
    return;
end

%% Overall
pr(rpt, '\n================ OVERALL (%d spots, %d sweeps) ================', ...
   numel(spots), numel(allErr));
rmseAll = 1000 * sqrt(mean(allErr.^2));
pr(rpt, '2D error: RMSE %.0f | median %.0f | p95 %.0f | max %.0f mm', ...
   rmseAll, 1000 * median(allErr), 1000 * prctile(allErr, 95), 1000 * max(allErr));
if rmseAll <= 30
    pr(rpt, 'GOAL MET: RMSE %.0f mm <= 30 mm (static).', rmseAll);
else
    pr(rpt, 'Goal not met yet: RMSE %.0f mm > 30 mm — check the per-spot table.', rmseAll);
end

%% Map figure
fig = figure('Visible', 'off', 'Position', [0 0 950 600]);
hold on;
plot(A.pos(:, 1), A.pos(:, 2), 'k^', 'MarkerFaceColor', 'y', 'MarkerSize', 10);
text(A.pos(:, 1) + 0.06, A.pos(:, 2), compose('A%d', A.ids));
for j = 1:numel(spots)
    t = spots(j).truth; m = spots(j).medPos;
    plot(t(1), t(2), 'gx', 'MarkerSize', 12, 'LineWidth', 2);
    plot(m(1), m(2), 'r.', 'MarkerSize', 14);
    plot([t(1) m(1)], [t(2) m(2)], 'r-');
    text(t(1) + 0.05, t(2) + 0.05, sprintf('S%d: %.0f mm', spots(j).k, ...
         spots(j).stats.rmse_mm));
end
axis equal; grid on;
title(sprintf('Static accuracy — RMSE %.0f mm over %d spots (x = truth, . = median fix)', ...
      rmseAll, numel(spots)));
xlabel('x (m)'); ylabel('y (m)');
saveas(fig, fullfile(outDir, 'map.png'));
close(fig);

%% Save
out = struct('schema', 'dune_static_accuracy_v1', ...
             'timestamp', stamp, 'tagId', opts.tagId, 'tagZ', opts.tagZ, ...
             'overall_rmse_mm', round(rmseAll, 1), 'spots', spots);
fh = fopen(fullfile(outDir, 'spots.json'), 'w');
fwrite(fh, jsonencode(out, 'PrettyPrint', true));
fclose(fh);
pr(rpt, '\nOutputs in %s', outDir);
end

%% ── helpers ────────────────────────────────────────────────────────────────
function sweeps = collectSweeps(ts, tagId, durS)
sweeps = {};
t0 = tic;
while toc(t0) < durS
    for c = ts.drain()
        if c{1}.tag == tagId
            sweeps{end+1} = c{1}; %#ok<AGROW>
        end
    end
    ts.drainEvents();
    pause(0.05);
end
end

function pr(fid, fmt, varargin)
fprintf(fid, [fmt '\n'], varargin{:});
fprintf([fmt '\n'], varargin{:});
end
