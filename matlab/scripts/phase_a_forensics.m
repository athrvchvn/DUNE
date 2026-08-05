function out = phase_a_forensics()
%PHASE_A_FORENSICS Wobble investigation, Phase A (offline, no hardware).
%   A1: realised inter-exchange cadence vs range residual (cadence hypothesis).
%   A2: range-space breathing decomposition (cross-anchor correlation per dwell:
%       common-mode => tag-side cause, independent => per-anchor/link cause).
%
%   Data: 19-spot campaign (results\static_accuracy_20260720_195637) with
%   Kinect truth, plus long parked dwells auto-detected in matlab\logs
%   serial_raw_*.log. Residuals are per-(dwell,anchor) demeaned so the spatial
%   bias field drops out; "fast" residual additionally removes a 2 s moving
%   median so per-exchange cadence effects separate from the slow breathing.
%
%   Outputs: results\phaseA_forensics_<stamp>\{report.txt, *.png, phaseA.mat}

campDir = fullfile(dune.rootDir(), 'results', 'static_accuracy_20260720_195637');
logDir  = fullfile(dune.rootDir(), 'logs');
liveLogs = ["serial_raw_20260720_211833.log"
            "serial_raw_20260720_212041.log"
            "serial_raw_20260720_213433.log"
            "serial_raw_20260720_213811.log"
            "serial_raw_20260720_193348.log"];

A  = dune.loadAnchors();
RC = dune.loadRangeCorrection();
M  = numel(A.ids);

stamp = char(datetime('now', Format='yyyyMMdd_HHmmss'));
outDir = fullfile(dune.rootDir(), 'results', ['phaseA_forensics_' stamp]);
mkdir(outDir);
rpt = fopen(fullfile(outDir, 'report.txt'), 'w');
closeRpt = onCleanup(@() fclose(rpt));
pr(rpt, 'Phase A forensics — %s', stamp);
pr(rpt, 'Anchors: %s | range corr: ON | tagZ 0.24', A.layout);

%% ── Campaign: parse, scheduler reconstruction, solve, dwell segmentation ──
S = parseRawLog(fullfile(campDir, 'serial_raw.log'));
pr(rpt, '\nCampaign log: %d sweeps, %d SCHED events, %d boots', ...
   numel(S.tms), numel(S.schedT), numel(S.bootT));

St = reconstructScheduler(S, A.ids);          % status: N x M  1=valid 2=fail 3=skip
nPredSkip = sum(St.skipStart(:));
pr(rpt, 'Scheduler reconstruction: %d skip-window starts predicted vs %d [SCHED] skip lines', ...
   nPredSkip, S.nSchedSkip);

[P, S.corr] = solveAll(S, A, RC);             % positions + corrected ranges (N x M)

spec = jsondecode(fileread(fullfile(campDir, 'spots.json')));
nSpots = numel(spec.spots);
truthXY = reshape([spec.spots.truth], 2, [])';
% Segment against TRUTH, not the stored medPos: the campaign ran before
% range_correction.json existed, so its medPos are raw-solve medians (up to
% 0.33 m of bias-field offset); our corrected re-solve lands near truth.
dwell = segmentDwells(P, truthXY, S.thost);   % N x 1 spot label (0 = none)
cnt = histcounts(dwell(dwell > 0), 0.5:1:nSpots+0.5);
pr(rpt, 'Dwell segmentation: matched %d/%d spots; sweep counts vs spots.json:', ...
   sum(cnt > 0), nSpots);
pr(rpt, '  found  : %s', mat2str(cnt));
pr(rpt, '  expected: %s', mat2str([spec.spots.n]));

% Residuals vs truth (3D true range, corrected range).
tagZ = 0.24;
resid = nan(numel(S.tms), M);                 % corrected residual, m
for k = 1:nSpots
    idx = find(dwell == k);
    if isempty(idx), continue; end
    trueR = vecnorm(A.pos - [truthXY(k,:), tagZ], 2, 2)';
    resid(idx, :) = S.corr(idx, :) - trueR;
end

% Campaign observation table.
T1 = buildObsTable(S, St, resid, dwell, 'camp');

%% ── Live logs: parse, solve, auto-detect parked dwells, self-residuals ──
Tlive = table();
nextDwellId = 100;
for f = liveLogs'
    fp_ = fullfile(logDir, f);
    if ~isfile(fp_), pr(rpt, 'live log missing: %s', f); continue; end
    L = parseRawLog(fp_);
    if numel(L.tms) < 100, continue; end
    Lt = reconstructScheduler(L, A.ids);
    [Pl, L.corr] = solveAll(L, A, RC);
    dw = detectStillDwells(Pl, L.thost, nextDwellId);
    nDw = numel(unique(dw(dw > 0)));
    pr(rpt, 'live %s: %d sweeps, %d still dwells >=45 s', f, numel(L.tms), nDw);
    if nDw == 0, continue; end
    % self-residual: range - per-(dwell,anchor) median (no truth needed)
    res = nan(numel(L.tms), M);
    for d = unique(dw(dw > 0))'
        idx = dw == d;
        res(idx, :) = L.corr(idx, :) - median(L.corr(idx, :), 1, 'omitnan');
    end
    Tlive = [Tlive; buildObsTable(L, Lt, res, dw, char(f))]; %#ok<AGROW>
    nextDwellId = nextDwellId + nDw + 1;
end
T = [T1; Tlive];
pr(rpt, '\nPooled observations: %d (campaign %d, live %d)', ...
   height(T), height(T1), height(Tlive));

%% ── A1: cadence vs residual ──────────────────────────────────────────────
pr(rpt, '\n================ A1: cadence vs residual ================');
pr(rpt, ['Residuals demeaned per (dwell,anchor); "fast" = minus 2 s moving median.\n' ...
         'prevState: 1=first attempt of sweep (after inter-sweep idle ~50-120 ms),\n' ...
         '           2=preceded by a SUCCESS (~18 ms gap), 3=preceded by a FAIL (~50-80 ms gap)']);
a1 = struct();
for c = 1:M
    rows = T.anchor == A.ids(c) & isfinite(T.dresid);
    tt = T(rows, :);
    if height(tt) < 200, continue; end
    % gap since this anchor's own last success (s)
    [rhoOwn, pOwn]   = corr(tt.gapOwn,  tt.dresid, Type='Spearman', Rows='complete');
    [rhoDt,  pDt]    = corr(tt.dtSweep, tt.dresid, Type='Spearman', Rows='complete');
    [rhoOwnF, pOwnF] = corr(tt.gapOwn,  tt.fresid, Type='Spearman', Rows='complete');
    % categorical contrast on the FAST residual
    g1 = tt.fresid(tt.prevState == 1);   % first attempt after idle
    g2 = tt.fresid(tt.prevState == 2);   % back-to-back after success
    g3 = tt.fresid(tt.prevState == 3);   % after a failed slot (timeout gap)
    d12 = 1000 * (mean(g1, 'omitnan') - mean(g2, 'omitnan'));
    d32 = 1000 * (mean(g3, 'omitnan') - mean(g2, 'omitnan'));
    p12 = safeTtest(g1, g2);  p32 = safeTtest(g3, g2);
    % first sample after a long own-gap (>2 s) vs steady
    gLong = tt.fresid(tt.gapOwn > 2);
    dLong = 1000 * (mean(gLong, 'omitnan') - mean(tt.fresid(tt.gapOwn <= 0.4), 'omitnan'));
    pLong = safeTtest(gLong, tt.fresid(tt.gapOwn <= 0.4));
    pr(rpt, ['A%d (n=%d, n1/n2/n3=%d/%d/%d):\n' ...
             '  corr(dresid, ownGap) %+0.3f (p %.1e) | corr(dresid, sweepDt) %+0.3f (p %.1e) | corr(fast, ownGap) %+0.3f (p %.1e)\n' ...
             '  firstAttempt-vs-afterSuccess %+0.1f mm (p %.1e) | afterFail-vs-afterSuccess %+0.1f mm (p %.1e)\n' ...
             '  first-sample-after-gap>2s vs steady %+0.1f mm (p %.1e, n=%d)'], ...
       A.ids(c), height(tt), numel(g1), numel(g2), numel(g3), ...
       rhoOwn, pOwn, rhoDt, pDt, rhoOwnF, pOwnF, d12, p12, d32, p32, ...
       dLong, pLong, numel(gLong));
    a1(c).id = A.ids(c); a1(c).rho = [rhoOwn rhoDt rhoOwnF];
    a1(c).d12 = d12; a1(c).d32 = d32; a1(c).dLong = dLong;
    a1(c).p = [pOwn pDt pOwnF p12 p32 pLong];
end

figA1 = figure(Visible='off', Position=[0 0 1200 500]);
tiledlayout(1, M, TileSpacing='compact');
for c = 1:M
    nexttile; hold on; grid on;
    rows = T.anchor == A.ids(c) & isfinite(T.fresid);
    tt = T(rows, :);
    vals = {1000*tt.fresid(tt.prevState==1), 1000*tt.fresid(tt.prevState==2), ...
            1000*tt.fresid(tt.prevState==3)};
    for g = 1:3
        if isempty(vals{g}), continue; end
        mu = mean(vals{g}, 'omitnan');
        se = std(vals{g}, 'omitnan') / max(1, sqrt(sum(isfinite(vals{g}))));
        bar(g, mu, FaceAlpha=0.6);
        errorbar(g, mu, 1.96*se, 'k', LineWidth=1.2);
    end
    xticks(1:3); xticklabels({'idle', 'succ', 'fail'});
    title(sprintf('A%d', A.ids(c))); ylabel('fast resid mean (mm)');
end
sgtitle('A1: fast residual by preceding-slot state (95% CI)');
saveas(figA1, fullfile(outDir, 'a1_prevstate.png')); close(figA1);

figA1b = figure(Visible='off', Position=[0 0 1200 500]);
tiledlayout(1, M, TileSpacing='compact');
edges = [0.1 0.2 0.3 0.5 1 2 5 15 60];
for c = 1:M
    nexttile; hold on; grid on;
    rows = T.anchor == A.ids(c) & isfinite(T.dresid) & isfinite(T.gapOwn);
    tt = T(rows, :);
    [~, ~, bin] = histcounts(tt.gapOwn, edges);
    mu = nan(1, numel(edges)-1); se = mu;
    for b = 1:numel(edges)-1
        v = 1000 * tt.dresid(bin == b);
        if numel(v) < 10, continue; end
        mu(b) = mean(v); se(b) = std(v) / sqrt(numel(v));
    end
    ctr = sqrt(edges(1:end-1) .* edges(2:end));
    errorbar(ctr, mu, 1.96*se, 'o-');
    set(gca, XScale='log'); title(sprintf('A%d', A.ids(c)));
    xlabel('own inter-success gap (s)'); ylabel('dresid (mm)');
end
sgtitle('A1: residual vs realised per-anchor gap');
saveas(figA1b, fullfile(outDir, 'a1_gap_binned.png')); close(figA1b);

%% ── A2: breathing decomposition ──────────────────────────────────────────
pr(rpt, '\n================ A2: breathing decomposition ================');
dwIds = unique(T.dwell(T.dwell > 0))';
Csum = zeros(M); Cn = zeros(M);
pc1Share = []; pc1SameSign = []; dwellLen = []; dwUsed = [];
for d = dwIds
    tt = T(T.dwell == d, :);
    sw = unique(tt.sweepIdx);
    if numel(sw) < 60, continue; end
    R = nan(numel(sw), M);
    for c = 1:M
        rows = tt.anchor == A.ids(c);
        [tf, loc] = ismember(tt.sweepIdx(rows), sw);
        v = tt.sresid(rows);                 % 2 s-median smoothed residual
        R(loc(tf), c) = v(tf);
    end
    keep = mean(isfinite(R), 1) > 0.8;       % anchors present >80% of dwell
    if sum(keep) < 3, continue; end
    Rk = R(:, keep);
    Rk = fillmissing(Rk, 'linear', EndValues='nearest');
    % pairwise correlations
    Cd = corr(Rk);
    ki = find(keep);
    for i = 1:numel(ki)
        for j = 1:numel(ki)
            Csum(ki(i), ki(j)) = Csum(ki(i), ki(j)) + Cd(i, j);
            Cn(ki(i), ki(j)) = Cn(ki(i), ki(j)) + 1;
        end
    end
    % PC1 share of slow variance
    Rc = Rk - mean(Rk, 1);
    [~, Sv, V] = svd(Rc, 'econ');
    ev = diag(Sv).^2;
    pc1Share(end+1) = ev(1) / sum(ev); %#ok<AGROW>
    pc1SameSign(end+1) = all(V(:,1) > 0) || all(V(:,1) < 0); %#ok<AGROW>
    dwellLen(end+1) = numel(sw); %#ok<AGROW>
    dwUsed(end+1) = d; %#ok<AGROW>
end
Cavg = Csum ./ max(Cn, 1);
pr(rpt, 'Dwells used: %d (>=60 sweeps, >=3 anchors)', numel(pc1Share));
pr(rpt, 'Mean pairwise cross-anchor corr of smoothed residuals (off-diag):');
for c = 1:M
    pr(rpt, '  A%d: %s', A.ids(c), sprintf('%+6.2f ', Cavg(c, :)));
end
offd = Cavg(~eye(M) & Cn > 0);
pr(rpt, 'Off-diagonal mean %+0.2f (common-mode if >> 0, independent if ~0)', ...
   mean(offd, 'omitnan'));
pr(rpt, 'PC1 slow-variance share: median %.0f%% (uniform-noise baseline ~%.0f%%); PC1 loadings same-sign in %.0f%% of dwells', ...
   100*median(pc1Share), 100/4, 100*mean(pc1SameSign));

figA2 = figure(Visible='off', Position=[0 0 500 420]);
imagesc(Cavg, [-1 1]); colorbar; axis square;
xticks(1:M); yticks(1:M);
xticklabels(compose('A%d', A.ids)); yticklabels(compose('A%d', A.ids));
title('A2: mean cross-anchor corr of slow residuals');
saveas(figA2, fullfile(outDir, 'a2_crosscorr.png')); close(figA2);

% Example dwell overlays (longest 4 dwells)
[~, ord] = sort(dwellLen, 'descend');
figA2b = figure(Visible='off', Position=[0 0 1200 700]);
tiledlayout(2, 2, TileSpacing='compact');
usable = dwUsed(ord(1:min(4, numel(ord))));
for d = usable
    nexttile; hold on; grid on;
    tt = T(T.dwell == d, :);
    for c = 1:M
        rows = tt.anchor == A.ids(c);
        if sum(rows) < 20, continue; end
        plot(tt.t(rows) - min(tt.t), 1000 * tt.sresid(rows), '.-', MarkerSize=4, ...
             DisplayName=sprintf('A%d', A.ids(c)));
    end
    xlabel('t (s)'); ylabel('slow resid (mm)'); legend(Location='eastoutside');
    title(sprintf('dwell %d', d));
end
sgtitle('A2: per-anchor slow residual time series (2 s median)');
saveas(figA2b, fullfile(outDir, 'a2_dwell_examples.png')); close(figA2b);

save(fullfile(outDir, 'phaseA.mat'), 'T', 'a1', 'Cavg', 'Cn', 'pc1Share', ...
     'pc1SameSign', 'dwell', 'P', '-v7.3');
pr(rpt, '\nOutputs in %s', outDir);
out = struct('dir', outDir, 'a1', a1, 'Cavg', Cavg, 'pc1Share', pc1Share);
end

%% ════════════════════════════ helpers ════════════════════════════════════

function S = parseRawLog(file)
% Parse a tab-separated <host_ts>\t<payload> raw serial log.
% Returns sweeps (tms, thost, per-anchor dist/rx/fp in full M columns is done
% later — here raw token arrays), SCHED skip events, boot markers.
lines = readlines(file);
n = numel(lines);
S.tms = []; S.thost = []; S.ids = {}; S.dist = {}; S.rx = {}; S.fp = {};
S.schedT = []; S.schedAddr = []; S.schedKind = [];  % 1=skip 2=recovered
S.bootT = [];
S.nSchedSkip = 0;
for i = 1:n
    ln = lines(i);
    tab = strfind(ln, sprintf('\t'));
    if isempty(tab), continue; end
    th = str2double(extractBefore(ln, tab(1)));
    pay = extractAfter(ln, tab(1));
    if startsWith(pay, 'RTLS,')
        s = dune.parseRtlsLine(pay);
        if isempty(s), continue; end
        S.tms(end+1, 1) = s.tms; S.thost(end+1, 1) = th;
        S.ids{end+1, 1} = s.ids; S.dist{end+1, 1} = s.dist;
        S.rx{end+1, 1} = s.rx;  S.fp{end+1, 1} = s.fp;
    elseif contains(pay, '[SCHED]')
        tok = regexp(pay, 'anchor 0x([0-9A-Fa-f]+): \d+ fails -> skip (\d+)', 'tokens', 'once');
        if ~isempty(tok)
            S.schedT(end+1,1) = th;
            S.schedAddr(end+1,1) = hex2dec(tok{1});
            S.schedKind(end+1,1) = 1;
            S.nSchedSkip = S.nSchedSkip + 1;
        elseif contains(pay, 'recovered')
            tok = regexp(pay, 'anchor 0x([0-9A-Fa-f]+) recovered', 'tokens', 'once');
            if ~isempty(tok)
                S.schedT(end+1,1) = th;
                S.schedAddr(end+1,1) = hex2dec(tok{1});
                S.schedKind(end+1,1) = 2;
            end
        end
    elseif contains(pay, '[BOOT]')
        S.bootT(end+1, 1) = th;
    end
end
end

function St = reconstructScheduler(S, ids)
% Replicate UwbScheduler fail/skip state machine over the observed sweeps.
% status(i,c): 1=valid, 2=fail(attempted, timed out), 3=skipped(backoff).
% Also marks skip-window starts for cross-validation against [SCHED] lines.
SKIP_AFTER_FAILS = 2; SKIP_BASE = 2; SKIP_MAX = 40;
N = numel(S.tms); M = numel(ids);
status = zeros(N, M); skipStart = false(N, M);
fails = zeros(1, M); skips = zeros(1, M); mult = ones(1, M);
lastTms = -inf;
for i = 1:N
    if S.tms(i) < lastTms   % tag reboot: state machine resets
        fails(:) = 0; skips(:) = 0; mult(:) = 1;
    end
    lastTms = S.tms(i);
    for c = 1:M
        present = ismember(ids(c), S.ids{i});
        if skips(c) > 0 && ~present
            skips(c) = skips(c) - 1;
            status(i, c) = 3;
            continue;
        end
        if present
            status(i, c) = 1;
            fails(c) = 0; mult(c) = 1; skips(c) = 0;
        else
            status(i, c) = 2;
            fails(c) = fails(c) + 1;
            if fails(c) >= SKIP_AFTER_FAILS
                skips(c) = min(SKIP_BASE * mult(c), SKIP_MAX);
                fails(c) = 0;
                mult(c) = min(mult(c) * 2, 8);
                skipStart(i, c) = true;
            end
        end
    end
end
St.status = status; St.skipStart = skipStart;
end

function [P, corrM] = solveAll(S, A, RC)
% Cold-start every solve: a warm-start chain running through carrying phases
% can lock the gated LM into a shifted local solution for a whole dwell
% (bad x0 -> MAD gate drops a good anchor persistently). Cold start is
% unbiased, which segmentation depends on.
N = numel(S.tms);
P = nan(N, 2);
corrM = nan(N, numel(A.ids));
for i = 1:N
    sw = struct('tag', 240, 'ids', S.ids{i}, 'dist', S.dist{i}, ...
                'rx', S.rx{i}, 'fp', S.fp{i});
    [p, info] = dune.solveSweep(sw, A, rangeCorr=RC, tagZ=0.24);
    P(i, :) = p;
    corrM(i, :) = info.rangeCorr';
end
end

function dwell = segmentDwells(P, truthXY, thost)
% Chronological dwell windows. For each spot k (visited in order): cluster the
% sweeps whose corrected solve lands near truth_k (after the previous dwell)
% into time windows and claim the FULL index span of the earliest cluster
% whose median position matches truth_k - unsolved sweeps inside a parked
% window are still parked (range residuals do not need the solve). The median
% check keeps near-coincident spots (9 vs 13 at 0.24 m) from cross-claiming.
N = size(P, 1); K = size(truthXY, 1);
lbl = zeros(N, 1);
for i = 1:N
    if ~isfinite(P(i, 1)), continue; end
    d = vecnorm(truthXY - P(i, :), 2, 2);
    [dm, k] = min(d);
    if dm < 0.30, lbl(i) = k; end
end
dwell = zeros(N, 1);
lastEnd = 0;
for k = 1:K
    m = find(lbl == k);
    m = m(m > lastEnd);
    if numel(m) < 20, continue; end
    cl = cumsum([1; diff(thost(m)) > 30]);
    for b = 1:max(cl)
        w = m(cl == b);
        if numel(w) < 20, continue; end
        seg = w(1):w(end);
        solved = seg(isfinite(P(seg, 1)));
        if isempty(solved) || norm(median(P(solved, :), 1) - truthXY(k, :)) > 0.20
            continue;                    % window centre disagrees with the spot
        end
        seg = seg(4:end-3);              % trim settle/pickup edges
        dwell(seg) = k;
        lastEnd = seg(end);
        break;
    end
end
end

function dw = detectStillDwells(P, thost, baseId)
% Still dwell: 2 s-median position stays within 0.05 m of its local value for
% >= 45 s. Label contiguous still stretches with unique ids starting baseId.
N = size(P, 1);
dw = zeros(N, 1);
if N < 100, return; end
Pm = movmedian(P, 11, 1, 'omitnan');
v = [0; vecnorm(diff(Pm), 2, 2)] ./ max([1; diff(thost)], 1e-3);
still = v < 0.06 & isfinite(Pm(:, 1));      % < 6 cm/s of smoothed motion
still = movmin(movmax(double(still), 7), 7) > 0.5;   % morphological closing:
                                             % fill glitch gaps <= 6 samples so
                                             % outlier solves don't break runs
d = diff([0; still; 0]);
starts = find(d == 1); ends = find(d == -1) - 1;
id = baseId;
for s = 1:numel(starts)
    if thost(ends(s)) - thost(starts(s)) < 40, continue; end
    seg = starts(s):ends(s);
    % require the whole segment to hug its own median position
    ctr = median(Pm(seg, :), 1, 'omitnan');
    if max(vecnorm(Pm(seg, :) - ctr, 2, 2)) > 0.12, continue; end
    dw(seg(5:end-4)) = id;
    id = id + 1;
end
end

function T = buildObsTable(S, St, resid, dwell, srcName)
% One row per (sweep, valid anchor observation) with cadence covariates.
N = numel(S.tms); M = size(resid, 2);
ids = [1 2 3 4 5];
dtSweep = [nan; diff(S.tms)] / 1000;
dtSweep(dtSweep < 0) = nan;                  % reboot
rows = [];
lastOk = nan(1, M);
for i = 1:N
    attempted = 0; fails = 0; prevWasFail = false;
    for c = 1:M
        st = St.status(i, c);
        if st == 3, continue; end            % skipped: no exchange, no time
        if st == 2
            attempted = attempted + 1; fails = fails + 1;
            prevWasFail = true;
            continue;
        end
        % valid observation
        if attempted == 0, ps = 1; elseif prevWasFail, ps = 3; else, ps = 2; end
        gapOwn = nan;
        if isfinite(lastOk(c)), gapOwn = (S.tms(i) - lastOk(c)) / 1000; end
        rows(end+1, :) = [i, ids(c), dwell(i), S.thost(i), resid(i, c), ...
                          gapOwn, dtSweep(i), ps, attempted, fails]; %#ok<AGROW>
        attempted = attempted + 1;
        prevWasFail = false;
        lastOk(c) = S.tms(i);
    end
end
T = array2table(rows, VariableNames={'sweepIdx', 'anchor', 'dwell', 't', ...
    'resid', 'gapOwn', 'dtSweep', 'prevState', 'attemptPos', 'failsBefore'});
T.src = repmat(string(srcName), height(T), 1);
% demean per (dwell, anchor); smooth per (dwell, anchor) for slow component
T.dresid = nan(height(T), 1);
T.sresid = nan(height(T), 1);
T.fresid = nan(height(T), 1);
for d = unique(T.dwell(T.dwell > 0))'
    for c = ids
        r = T.dwell == d & T.anchor == c;
        if sum(r) < 10, continue; end
        v = T.resid(r);
        v = v - median(v, 'omitnan');
        sm = movmedian(v, 11, 'omitnan');    % ~2 s at 5-7 Hz
        T.dresid(r) = v;
        T.sresid(r) = sm;
        T.fresid(r) = v - sm;
    end
end
T = T(T.dwell > 0 & isfinite(T.resid), :);
end

function p = safeTtest(a, b)
a = a(isfinite(a)); b = b(isfinite(b));
if numel(a) < 5 || numel(b) < 5, p = nan; return; end
[~, p] = ttest2(a, b);
end

function pr(fid, fmt, varargin)
fprintf(fid, [fmt '\n'], varargin{:});
fprintf([fmt '\n'], varargin{:});
end
