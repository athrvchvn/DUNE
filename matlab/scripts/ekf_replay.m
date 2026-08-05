function out = ekf_replay(src, opts)
%EKF_REPLAY Run dune.FusionEkf offline over a logged session (step 7.2).
%   out = ekf_replay("D:\...\serial_raw.log")                 % raw serial tee
%   out = ekf_replay("D:\...\rtls_log_x.jsonl")               % live session log
%   out = ekf_replay(src, spotsJson="...\spots.json")         % + static eval
%
%   Chain per sweep: dune.solveSweep (power-corrected, NLOS-weighted LM fix
%   + covariance) -> FusionEkf predict (IMU-driven when the sweep carries a
%   BNO085 tail, CV otherwise) -> gated position update -> ZUPT when the IMU
%   says the unit is still.
%
%   With spotsJson (static_accuracy campaign): clusters sweeps to the known
%   spots, drops the first settleS seconds of each dwell, and reports raw
%   per-sweep vs EKF error per spot and overall.

arguments
    src (1,1) string
    opts.tagId (1,1) double = 240
    opts.tagZ (1,1) double = 0.24
    opts.powerCorr (1,1) logical = true
    opts.useImu (1,1) logical = true       % IMU stillness detection -> ZUPT
    opts.useImuAccel (1,1) logical = false % IMU-driven prediction (frame check pending, 7.3)
    opts.mode string = "pos"               % "pos" = position-fix updates (default,
                                           % robust); "ranges" = tightly-coupled
                                           % per-range updates (EXPERIMENTAL: on the
                                           % static campaign it underperforms - it
                                           % needs per-range timing + moving truth
                                           % (7.5) to show its motion-coherence
                                           % advantage, and stricter ZUPT first)
    opts.sigmaR (1,1) double = 0.05        % per-range meas sigma (ranges mode)
    opts.robust (1,1) logical = true       % Huber down-weighting vs hard gate
    opts.stillInvert (1,1) logical = false % Phase-A3: while still, position Q ~0
                                           % + per-anchor bias RW opened (ranges
                                           % mode only) so breathing is absorbed
                                           % into the bias states
    opts.spotsJson string = ""
    opts.settleS (1,1) double = 2
    opts.sigmaAccelCV double = []      % override FusionEkf defaults if set
    opts.sigmaAccel double = []
    opts.quiet (1,1) logical = false
end

A = dune.loadAnchors();
RC = [];
if opts.powerCorr, RC = dune.loadRangeCorrection(); end

%% Load sweeps (+thost) from either source format
sw = {};
if endsWith(src, ".jsonl")
    S = dune.readSessionLog(src);
    T = S.tab(S.tab.tag == opts.tagId, :);
    for i = 1:height(T)
        have = find(~isnan(T.R(i, :)));
        if isempty(have), continue; end
        s = struct('tms', T.tms(i), 'tag', T.tag(i), ...
            'ids', S.meta.anchorIds(have), 'dist', T.R(i, have), ...
            'rx', T.RX(i, have), 'fp', T.FP(i, have), ...
            'qual', nan(1, numel(have)), 'imu', [], 'thost', T.thost(i));
        sw{end+1} = s; %#ok<AGROW>
    end
else
    raw = readlines(src);
    for i = 1:numel(raw)
        parts = split(raw(i), sprintf('\t'));
        if numel(parts) < 2, continue; end
        s = dune.parseRtlsLine(parts(2));
        if isempty(s) || s.tag ~= opts.tagId, continue; end
        s.thost = double(parts(1));
        sw{end+1} = s; %#ok<AGROW>
    end
end
N = numel(sw);
assert(N > 0, 'no sweeps for tag %d in %s', opts.tagId, src);
if ~opts.quiet, fprintf('%d sweeps (tag %d) from %s\n', N, opts.tagId, src); end

%% Run the filter
ekf = dune.FusionEkf();
ekf.robust = opts.robust;
if ~isempty(opts.sigmaAccelCV), ekf.sigmaAccelCV = opts.sigmaAccelCV; end
if ~isempty(opts.sigmaAccel),   ekf.sigmaAccel = opts.sigmaAccel; end
if opts.mode == "ranges"
    ekf.enableRangeBias(numel(A.ids));   % per-anchor bias memory
end

Praw = nan(N, 2); Pekf = nan(N, 2); thost = nan(N, 1);
imuMode = false(N, 1); zupt = false(N, 1); accepted = true(N, 1);
prevRaw = [];
tPrev = NaN;
stillT0 = NaN;
divergeStreak = 0;
histA = nan(1, 8); histG = nan(1, 8);   % rolling |acc| / |gyro| for stillness
for i = 1:N
    s = sw{i};
    thost(i) = s.thost;
    dt = 0;
    if isfinite(tPrev), dt = min(max(s.thost - tPrev, 0), 1); end
    tPrev = s.thost;

    aW = [];
    still = false;
    if opts.useImu && ~isempty(s.imu) && s.imu.status >= 1
        histA = [histA(2:end), norm(s.imu.acc)];
        histG = [histG(2:end), norm(s.imu.gyro)];
        % Still only if the WHOLE recent window is quiet - instantaneous
        % |acc| dips below any threshold mid-stride, a window does not.
        still = all(isfinite(histA)) && max(histA) < 0.12 && max(histG) < 0.05;
        if opts.useImuAccel
            aW = dune.FusionEkf.worldAccel(s.imu.quat, s.imu.acc);
        end
    end
    imuMode(i) = ~isempty(aW);
    ekf.stillMode = still;      % still -> ZUPT + frozen process noise
    % Phase-A3 inversion engages only after 3 s of continuous stillness: the
    % filter must have converged onto the parked position BEFORE the freeze,
    % otherwise the initial position error is absorbed into the bias states
    % (position/bias common null space) and never corrects.
    if still
        if ~isfinite(stillT0), stillT0 = s.thost; end
    else
        stillT0 = NaN;
    end
    ekf.stillInvert = opts.stillInvert && isfinite(stillT0) ...
                      && s.thost - stillT0 > 3;
    ekf.predict(dt, aW);

    [p, info] = dune.solveSweep(s, A, rangeCorr=RC, tagZ=opts.tagZ, x0=prevRaw);
    Praw(i, :) = p;
    if all(isfinite(p)), prevRaw = p; end

    if opts.mode == "ranges"
        if ~ekf.initialized
            if all(isfinite(p)), ekf.updatePosition(p); end   % first fix = init
        else
            nAcc = ekf.updateRanges(A.pos, info.rangeCorr, info.w, ...
                                    opts.tagZ, opts.sigmaR);
            accepted(i) = nAcc > 0;
            % Recovery nets. Partial acceptance can keep a tightly-coupled
            % filter alive at a WRONG position (1-2 ranges still fit), so a
            % reject-streak alone is not enough: also re-anchor when the EKF
            % persistently disagrees with a solid standalone fix.
            solid = all(isfinite(p)) && nnz(info.used) >= 4 && info.rmse < 0.10;
            if solid && norm(ekf.pos - p) > 0.5
                divergeStreak = divergeStreak + 1;
            elseif solid
                divergeStreak = 0;
            end
            if (ekf.consecReject >= ekf.maxConsecReject || divergeStreak >= 5) ...
                    && all(isfinite(p))
                ekf.reinitFrom(p);
                divergeStreak = 0;
            end
        end
    else
        if all(isfinite(p))
            accepted(i) = ekf.updatePosition(p, adaptiveR(info, ekf.posSigma));
        end
    end
    if still && ekf.initialized
        ekf.updateZupt();
        zupt(i) = true;
    end
    if ekf.initialized, Pekf(i, :) = ekf.pos; end
end
if ~opts.quiet
    fprintf('IMU-driven predicts: %.0f%%   ZUPT sweeps: %.0f%%   gated-out fixes: %d   reinits: %d\n', ...
        100 * mean(imuMode), 100 * mean(zupt), ekf.nRejected, ekf.nReinit);
end

out = struct('thost', thost, 'Praw', Praw, 'Pekf', Pekf, ...
             'imuMode', imuMode, 'zupt', zupt, 'accepted', accepted);

%% Track figure
[dirp, base] = fileparts(src);
fig = figure('Visible', 'off', 'Position', [0 0 950 600]);
hold on;
plot(Praw(:, 1), Praw(:, 2), '.', 'MarkerSize', 4, 'Color', [0.75 0.75 0.75], ...
     'DisplayName', 'raw per-sweep');
plot(Pekf(:, 1), Pekf(:, 2), '-', 'LineWidth', 1.2, 'Color', [0.85 0.2 0.2], ...
     'DisplayName', 'EKF');
plot(A.pos(:, 1), A.pos(:, 2), 'k^', 'MarkerFaceColor', 'y', 'DisplayName', 'anchors');
axis equal; grid on; legend('Location', 'bestoutside');
title(sprintf('ekf\\_replay — %s', base), 'Interpreter', 'tex');
figFile = fullfile(dirp, sprintf('ekf_replay_%s.png', base));
saveas(fig, figFile);
close(fig);
out.figFile = figFile;
if ~opts.quiet, fprintf('Track figure: %s\n', figFile); end

%% Static-spot evaluation
if opts.spotsJson == "", return; end
SP = jsondecode(fileread(opts.spotsJson));
sp = SP.spots([SP.spots.k] ~= 20);          % spot 20 excluded by user
nS = numel(sp);
allRaw = []; allEkf = [];
if ~opts.quiet
    fprintf('\nStatic-spot eval (settle %.0f s dropped per dwell):\n', opts.settleS);
    fprintf('  spot    n    raw RMSE   EKF RMSE   EKF median   EKF p95 (mm)   wander raw->EKF (mm)\n');
end
perSpot = nan(nS, 2);
perWander = nan(nS, 2);
for j = 1:nS
    t = sp(j).truth(:)';
    % cluster on TRUTH: spots.json medPos are pre-correction raw-solve medians
    % (up to 0.33 m of bias-field offset); corrected solves land near truth
    m = find(~isnan(Praw(:, 1)) & vecnorm(Praw - t, 2, 2) < 0.20)';
    if isempty(m), continue; end
    % contiguous dwells; drop the first settleS of each
    keepIdx = [];
    runStart = m(1);
    for q = 2:numel(m) + 1
        if q > numel(m) || thost(m(q)) - thost(m(q - 1)) > 2
            run = m(runStart <= m & m <= m(q - 1));
            keepIdx = [keepIdx, run(thost(run) >= thost(run(1)) + opts.settleS)]; %#ok<AGROW>
            if q <= numel(m), runStart = m(q); end
        end
    end
    if isempty(keepIdx), continue; end
    er = vecnorm(Praw(keepIdx, :) - t, 2, 2);
    ee = vecnorm(Pekf(keepIdx, :) - t, 2, 2);
    ee = ee(~isnan(ee));
    perSpot(j, :) = 1000 * [sqrt(mean(er.^2)), sqrt(mean(ee.^2))];
    % Wander = slow drift of the 2 s-median position within the dwell (the
    % breathing metric; RMSE alone cannot separate offset from wander).
    perWander(j, 1) = wanderMm(Praw(keepIdx, :));
    perWander(j, 2) = wanderMm(Pekf(keepIdx, :));
    if ~opts.quiet
        fprintf('  S%-3d %5d   %7.0f    %7.0f     %7.0f    %7.0f      %5.0f -> %5.0f\n', ...
            sp(j).k, numel(keepIdx), perSpot(j, 1), perSpot(j, 2), ...
            1000 * median(ee), 1000 * prctile(ee, 95), ...
            perWander(j, 1), perWander(j, 2));
    end
    allRaw = [allRaw; er]; allEkf = [allEkf; ee]; %#ok<AGROW>
end
out.spotRmse = perSpot;
out.spotWander = perWander;
out.overall = 1000 * [sqrt(mean(allRaw.^2)), sqrt(mean(allEkf.^2))];
out.wander = median(perWander, 1, 'omitnan');
if ~opts.quiet
    fprintf('  OVERALL: raw RMSE %4.0f -> EKF RMSE %4.0f mm   (median %4.0f -> %4.0f, p95 %4.0f -> %4.0f)\n', ...
        out.overall(1), out.overall(2), 1000 * median(allRaw), 1000 * median(allEkf), ...
        1000 * prctile(allRaw, 95), 1000 * prctile(allEkf, 95));
    fprintf('  WANDER (median over spots, max drift of 2 s-median pos): raw %.0f -> EKF %.0f mm\n', ...
        out.wander(1), out.wander(2));
end
end

function w = wanderMm(P)
% Max drift of the 2 s-median position within a dwell (breathing metric).
P = P(isfinite(P(:, 1)), :);
if size(P, 1) < 20, w = NaN; return; end
Pm = movmedian(P, 11, 1);
w = 1000 * max(vecnorm(Pm - median(Pm, 1), 2, 2));
end

%% ── helpers ────────────────────────────────────────────────────────────────
function R = adaptiveR(info, posSigma)
% Port of the old runtime's _adaptive_R: LM covariance inflated by solver
% RMSE, mean NLOS gap, and a DOP proxy (clipped 1..50x).
if all(isfinite(info.cov(:)))
    Rb = info.cov + eye(2) * 0.02^2;
else
    Rb = eye(2) * posSigma^2;
end
rmsF = 1 + (info.rmse / 0.05)^2;
g = info.gap(isfinite(info.gap) & isfinite(info.range));
if isempty(g), nlosF = 1; else, nlosF = 1 + mean(max(0, g / 6)); end
dop = sqrt(trace(Rb));
dopF = 1 + max(0, (dop - 0.05) / 0.05);
R = Rb * min(max(rmsF * nlosF * dopF, 1), 50);
end
