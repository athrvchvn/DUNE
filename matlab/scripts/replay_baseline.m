function out = replay_baseline(logFile, opts)
%REPLAY_BASELINE Re-solve a logged session from raw ranges; report accuracy.
%   out = replay_baseline(logFile)
%   out = replay_baseline(logFile, visionCsv240=..., visionCsv241=..., ...)
%
%   Re-runs the weighted-LM solver on the RAW ranges of every sweep in a
%   JSONL session log and reports, per tag:
%     - update rate, per-anchor availability, NLOS gap statistics
%     - solver residual statistics (internal consistency)
%     - trajectory comparison against the original pipeline output
%     - if a Kinect trajectory CSV is supplied and the log has t_host:
%       true 2D RMSE / percentiles / per-zone map vs ground truth
%
%   Results (report.txt + figures) go to matlab\results\<session-name>\.

arguments
    logFile (1,1) string
    opts.visionCsv240 string = ""     % Kinect CSV for tag 240 (0xF0)
    opts.visionCsv241 string = ""     % Kinect CSV for tag 241 (0xF1)
    opts.lever240 (1,2) double = [0 0]
    opts.lever241 (1,2) double = [0 0]
    opts.tagZ (1,1) double = 0.22
    opts.useGapWeights (1,1) logical = true
    opts.anchors string = ""
end

A = dune.loadAnchors(ifelse(opts.anchors ~= "", char(opts.anchors), []));
S = dune.readSessionLog(logFile);

[~, sessName] = fileparts(logFile);
outDir = fullfile(dune.rootDir(), 'results', char(sessName));
if ~exist(outDir, 'dir'), mkdir(outDir); end
rpt = fopen(fullfile(outDir, 'report.txt'), 'w');
cleanup = onCleanup(@() fclose(rpt));

pr(rpt, 'DUNE baseline replay — %s', sessName);
pr(rpt, 'Log: %s', S.meta.file);
pr(rpt, 'Records: %d (bad lines: %d)   Duration: %.1f s   t_host: %s', ...
    S.meta.nRecords, S.meta.nBadLines, S.meta.durationS, string(S.meta.hasThost));
if ~isempty(S.events)
    lines = {S.events.line};
    nBrown = nnz(contains(lines, 'BROWNOUT'));
    nPanic = nnz(contains(lines, 'PANIC')) + nnz(contains(lines, 'WDT'));
    pr(rpt, 'Device boot events during session: %d total (%d BROWNOUT, %d PANIC/WDT)', ...
        numel(S.events), nBrown, nPanic);
    srcs = unique({S.events.src});
    for k = 1:numel(srcs)
        pr(rpt, '  %s: %d resets', srcs{k}, nnz(strcmp({S.events.src}, srcs{k})));
    end
end
pr(rpt, 'Anchors (%s):', A.layout);
for k = 1:numel(A.ids)
    pr(rpt, '  A%d  [%.2f  %.2f  %.2f]', A.ids(k), A.pos(k,:));
end

out.session = char(sessName);
out.tags = struct([]);

for tagId = S.meta.tags
    T = S.tab(S.tab.tag == tagId, :);
    N = height(T);
    pr(rpt, '\n================ TAG %d (0x%02X) — %d sweeps ================', ...
        tagId, tagId, N);

    % --- update rate ------------------------------------------------------
    if S.meta.hasThost, tt = T.thost; else, tt = T.tms / 1000; end
    dt = diff(tt); dt = dt(dt > 0 & dt < 5);
    pr(rpt, 'Update rate: median %.2f Hz  (mean interval %.0f ms, p95 %.0f ms)', ...
        1/median(dt), 1000*mean(dt), 1000*prctile(dt, 95));

    % --- per-anchor availability and NLOS gap -----------------------------
    pr(rpt, 'Per-anchor (raw ranges):');
    pr(rpt, '  id   avail%%   medGap dB   gap>3dB%%   gap>6dB%%   medRange m');
    for c = 1:numel(A.ids)
        have = ~isnan(T.R(:,c));
        g = T.GAP(have, c);
        pr(rpt, '  A%d   %5.1f     %6.2f      %5.1f      %5.1f      %6.2f', ...
            A.ids(c), 100*mean(have), median(g, 'omitnan'), ...
            100*mean(g > 3), 100*mean(g > 6), median(T.R(have,c)));
    end

    % --- re-solve every sweep from raw ranges ------------------------------
    P  = nan(N, 2);          % our LM position
    res = nan(N, numel(A.ids));
    rms = nan(N, 1);
    nUsed = nan(N, 1);
    prev = [];
    for i = 1:N
        r = T.R(i, :)';
        if opts.useGapWeights
            w = dune.gapWeights(T.GAP(i, :)');
        else
            w = ones(size(r));
        end
        [p, info] = dune.multilaterate(A.pos, r, weights=w, tagZ=opts.tagZ, x0=prev);
        P(i,:) = p;
        res(i,:) = info.resid';
        rms(i) = info.rmse;
        nUsed(i) = nnz(info.used);
        if all(isfinite(p)), prev = p; end
    end
    solved = ~isnan(P(:,1));
    pr(rpt, 'Replay solve: %.1f%% of sweeps solved  (median LM residual RMSE %.0f mm, p95 %.0f mm)', ...
        100*mean(solved), 1000*median(rms, 'omitnan'), 1000*prctile(rms, 95));

    % --- compare with the original pipeline output -------------------------
    both = solved & ~isnan(T.x);
    dPipe = hypot(P(both,1) - T.x(both), P(both,2) - T.y(both));
    pr(rpt, 'Difference vs original pipeline output: median %.0f mm, p95 %.0f mm', ...
        1000*median(dPipe), 1000*prctile(dPipe, 95));

    tagOut = struct('tag', tagId, 'P', P, 'lmRmse', rms, 'resid', res, ...
                    'nUsed', nUsed, 't', tt, 'metrics', []);

    % --- ground truth, if provided -----------------------------------------
    csv = "";
    lever = [0 0];
    if tagId == 240 && opts.visionCsv240 ~= "", csv = opts.visionCsv240; lever = opts.lever240; end
    if tagId == 241 && opts.visionCsv241 ~= "", csv = opts.visionCsv241; lever = opts.lever241; end
    if csv ~= ""
        if ~S.meta.hasThost
            pr(rpt, 'GROUND TRUTH SKIPPED: log has no t_host (recorded before 2026-07-02).');
        else
            V = dune.readVisionLog(csv);
            G = dune.alignTruth(V, T.thost, lever=lever);
            X = dune.trajectoryMetrics(P, G.xy, bounds=A.bounds(1:4));
            pr(rpt, 'GROUND TRUTH (%s, coverage %.0f%%):', csv, 100*G.coverage);
            pr(rpt, '  2D error: RMSE %.1f mm | mean %.1f | median %.1f | p95 %.1f | max %.1f  (n=%d)', ...
                1000*X.rmse, 1000*X.mean, 1000*X.median, 1000*X.p95, 1000*X.max, X.n);
            tagOut.metrics = X;
            fig = figure('Visible','off');
            imagesc(X.zone.xEdges, X.zone.yEdges, 1000*X.zone.rmse);
            axis xy equal tight; colorbar;
            title(sprintf('Tag %d zone RMSE (mm)', tagId));
            saveas(fig, fullfile(outDir, sprintf('zone_rmse_tag%d.png', tagId)));
            close(fig);
        end
    end

    % --- figures ------------------------------------------------------------
    fig = figure('Visible','off', 'Position', [0 0 900 500]);
    plot(T.x, T.y, '.', 'MarkerSize', 4, 'DisplayName', 'pipeline output'); hold on
    plot(P(:,1), P(:,2), '.', 'MarkerSize', 4, 'DisplayName', 'replay LM (raw ranges)');
    plot(A.pos(:,1), A.pos(:,2), 'k^', 'MarkerFaceColor', 'y', 'DisplayName', 'anchors');
    text(A.pos(:,1)+0.1, A.pos(:,2), compose('A%d', A.ids));
    axis equal; grid on; legend('Location','bestoutside');
    title(sprintf('%s — tag %d', sessName, tagId), 'Interpreter', 'none');
    saveas(fig, fullfile(outDir, sprintf('trajectory_tag%d.png', tagId)));
    close(fig);

    fig = figure('Visible','off', 'Position', [0 0 900 500]);
    for c = 1:numel(A.ids)
        subplot(numel(A.ids), 1, c);
        histogram(T.GAP(:,c), -1:0.25:15);
        xline(3, 'r-'); xline(6, 'r--');
        ylabel(sprintf('A%d', A.ids(c)));
        if c == 1, title(sprintf('rx-fp gap (dB), tag %d', tagId)); end
    end
    saveas(fig, fullfile(outDir, sprintf('gap_hist_tag%d.png', tagId)));
    close(fig);

    out.tags = [out.tags, tagOut]; %#ok<AGROW>
end

pr(rpt, '\nOutputs in %s', outDir);
fprintf('Report written to %s\n', fullfile(outDir, 'report.txt'));
out.outDir = outDir;
end

function pr(fid, fmt, varargin)
fprintf(fid, [fmt '\n'], varargin{:});
fprintf([fmt '\n'], varargin{:});
end

function v = ifelse(cond, a, b)
if cond, v = a; else, v = b; end
end
