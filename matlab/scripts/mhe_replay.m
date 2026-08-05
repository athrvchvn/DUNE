function out = mhe_replay(src, opts)
%MHE_REPLAY Offline A/B of the Moving Horizon Estimator vs the EKF (step 7 exp).
%   out = mhe_replay(fullfile(dune.logsDir(), "serial_raw_....log"))
%   out = mhe_replay(src, spotsJson="...\spots.json")   % + static wander/RMSE
%
%   Runs the identical solveSweep chain, then feeds each sweep to BOTH
%   dune.FusionEkf (pos mode, robust) and dune.MheEstimator, and compares:
%     - motion: path jerk (smoothness) + EKF/MHE-vs-raw offset (lag proxy)
%     - static (spotsJson): per-dwell scatter + 2 s-median wander + RMSE
%     - MHE per-sweep IPOPT solve time (live-feasibility check)
%   Produces a track figure (raw ghost, EKF, MHE) next to the log.

arguments
    src (1,1) string
    opts.tagId (1,1) double = 240
    opts.tagZ (1,1) double = 0.24
    opts.horizon (1,1) double = 10
    opts.sigmaAccel (1,1) double = 0.8
    opts.useImu (1,1) logical = false      % feed gyro yaw rate -> turn/heading
    opts.model string = "cv"               % "cv" | "unicycle" (non-holonomic)
    opts.spotsJson string = ""
    opts.casadiPath string = ""
    opts.plot (1,1) logical = true
end

dune.addCasadi(opts.casadiPath);
A = dune.loadAnchors();
RC = dune.loadRangeCorrection();

sw = loadSweeps(src, opts.tagId);
N = numel(sw);
assert(N > opts.horizon, 'only %d sweeps', N);
fprintf('%d sweeps (tag %d) from %s\n', N, opts.tagId, src);

ekf = dune.FusionEkf(); ekf.robust = true; ekf.sigmaAccelCV = opts.sigmaAccel;
mhe = dune.MheEstimator(A.pos);
mhe.horizon = opts.horizon; mhe.tagZ = opts.tagZ; mhe.sigmaAccel = opts.sigmaAccel;
mhe.model = opts.model;
mhe.build();

Praw = nan(N,2); Pekf = nan(N,2); Pmhe = nan(N,2); thost = nan(N,1);
solveT = nan(N,1);
prev = []; tPrev = NaN;
histA = nan(1,8); histG = nan(1,8);
for i = 1:N
    s = sw{i}; thost(i) = s.thost;
    dt = 0; if isfinite(tPrev), dt = min(max(s.thost - tPrev,0),1); end
    tPrev = s.thost;

    [p, info] = dune.solveSweep(s, A, rangeCorr=RC, tagZ=opts.tagZ, x0=prev);
    Praw(i,:) = p; if all(isfinite(p)), prev = p; end

    % stillness (for EKF ZUPT/stillMode parity with live)
    still = false;
    if ~isempty(s.imu) && s.imu.status >= 1
        histA = [histA(2:end), norm(s.imu.acc)];
        histG = [histG(2:end), norm(s.imu.gyro)];
        still = all(isfinite(histA)) && max(histA) < 0.12 && max(histG) < 0.05;
    end
    ekf.stillMode = still; ekf.predict(dt, []);
    if all(isfinite(p)), ekf.updatePosition(p, eye(2)*ekf.posSigma^2); end
    if still && ekf.initialized, ekf.updateZupt(); end
    if ekf.initialized, Pekf(i,:) = ekf.pos; end

    % gyro yaw rate (world-vertical) for the coordinated-turn model
    omega = 0;
    if opts.useImu && ~isempty(s.imu)
        omega = worldYawRate(s.imu.quat, s.imu.gyro);
    end
    % MHE (same stillness signal the EKF uses -> parity on ZUPT)
    tic;
    [pm, ~, mi] = mhe.push(info.rangeCorr, info.w, max(dt,1e-3), p, still, omega);
    solveT(i) = toc;
    if ~mi.warmup, Pmhe(i,:) = pm; elseif all(isfinite(p)), Pmhe(i,:) = p; end
end

st = solveT(~isnan(Pmhe(:,1)) & (1:N)' > opts.horizon);
fprintf('\nMHE solve time: median %.1f ms, p95 %.1f ms, max %.1f ms  (horizon %d); jump-guard trips: %d/%d\n', ...
    1000*median(st,'omitnan'), 1000*prctile(st,95), 1000*max(st), opts.horizon, ...
    mhe.nJumps, mhe.nSolves);

t = thost - thost(1);
sp = [0; vecnorm(diff(Pekf),2,2)] ./ max([1;diff(t)],1e-3);
mv = movmax(movmin(double(movmedian(sp,7) > 0.06),5),5) > 0.5;
fprintf('moving %.0f%% of the session\n', 100*mean(mv));

fprintf('\n            jerk(a.u.)   lag-vs-raw(mm, moving)\n');
fprintf('raw        %8.3f        %6s\n', jerk(Praw,t,mv), '-');
fprintf('EKF        %8.3f        %6.0f\n', jerk(Pekf,t,mv), offset(Pekf,Praw,mv));
fprintf('MHE        %8.3f        %6.0f\n', jerk(Pmhe,t,mv), offset(Pmhe,Praw,mv));

out = struct('t',t,'Praw',Praw,'Pekf',Pekf,'Pmhe',Pmhe,'moving',mv,'solveT',solveT);

% Static wander/RMSE per spot
if opts.spotsJson ~= ""
    SP = jsondecode(fileread(opts.spotsJson));
    sp_ = SP.spots([SP.spots.k] ~= 20);
    accum = struct('raw',[],'ekf',[],'mhe',[]);
    fprintf('\nStatic per-spot (truth-clustered): wander raw/EKF/MHE, RMSE raw/EKF/MHE (mm)\n');
    for j = 1:numel(sp_)
        tr = sp_(j).truth(:)';
        m = find(~isnan(Praw(:,1)) & vecnorm(Praw - tr,2,2) < 0.20)';
        if numel(m) < 30, continue; end
        m = m(t(m) >= t(m(1)) + 2);            % drop 2 s settle
        wr = wander(Praw(m,:)); we = wander(Pekf(m,:)); wm = wander(Pmhe(m,:));
        rr = rmse(Praw(m,:),tr); re = rmse(Pekf(m,:),tr); rm = rmse(Pmhe(m,:),tr);
        fprintf('  S%-2d n=%3d  wander %3.0f/%3.0f/%3.0f   RMSE %3.0f/%3.0f/%3.0f\n', ...
            sp_(j).k, numel(m), wr,we,wm, rr,re,rm);
        accum.raw(end+1,:) = [wr rr]; accum.ekf(end+1,:) = [we re]; accum.mhe(end+1,:) = [wm rm];
    end
    fprintf('  MEDIAN     wander %3.0f/%3.0f/%3.0f   RMSE %3.0f/%3.0f/%3.0f\n', ...
        median(accum.raw(:,1)), median(accum.ekf(:,1)), median(accum.mhe(:,1)), ...
        median(accum.raw(:,2)), median(accum.ekf(:,2)), median(accum.mhe(:,2)));
    out.static = accum;
end

if opts.plot
    fig = figure('Position',[0 0 1000 640]); hold on; grid on; axis equal;
    plot(Praw(:,1),Praw(:,2),'.','Color',[.75 .75 .75],'MarkerSize',4,'DisplayName','raw');
    plot(Pekf(:,1),Pekf(:,2),'-','Color',[.2 .4 .9],'LineWidth',1.1,'DisplayName','EKF');
    plot(Pmhe(:,1),Pmhe(:,2),'-','Color',[.85 .2 .2],'LineWidth',1.3,'DisplayName','MHE');
    plot(A.pos(:,1),A.pos(:,2),'k^','MarkerFaceColor','y','DisplayName','anchors');
    legend('Location','bestoutside'); xlabel('x (m)'); ylabel('y (m)');
    title(sprintf('MHE vs EKF  (horizon %d, MHE %.0f ms/solve)', ...
        opts.horizon, 1000*median(st,'omitnan')));
    f = replace(char(src),'.log','_mhe.png'); f = replace(f,'.jsonl','_mhe.png');
    saveas(fig, f); fprintf('Figure: %s\n', f);
    out.figFile = f;
end
end

%% ── helpers ──────────────────────────────────────────────────────────────
function sw = loadSweeps(src, tagId)
sw = {};
if endsWith(src, ".jsonl")
    S = dune.readSessionLog(src); T = S.tab(S.tab.tag == tagId, :);
    for i = 1:height(T)
        have = find(~isnan(T.R(i,:)));
        if isempty(have), continue; end
        sw{end+1} = struct('tms',T.tms(i),'tag',T.tag(i),'ids',S.meta.anchorIds(have), ...
            'dist',T.R(i,have),'rx',T.RX(i,have),'fp',T.FP(i,have), ...
            'qual',nan(1,numel(have)),'imu',[],'thost',T.thost(i)); %#ok<AGROW>
    end
else
    raw = readlines(src);
    for i = 1:numel(raw)
        parts = split(raw(i), sprintf('\t'));
        if numel(parts) < 2, continue; end
        s = dune.parseRtlsLine(parts(2));
        if isempty(s) || s.tag ~= tagId, continue; end
        s.thost = double(parts(1)); sw{end+1} = s; %#ok<AGROW>
    end
end
end

function j = jerk(P, t, mv)
P = P(mv,:); t = t(mv); ok = all(isfinite(P),2); P = P(ok,:); t = t(ok);
if size(P,1) < 6, j = NaN; return; end
P = movmedian(P,3,1); a = diff(P,2,1);
j = mean(vecnorm(diff(a,1,1),2,2),'omitnan') / max(median(diff(t)),1e-3);
end

function o = offset(P, R, mv)
o = 1000*median(vecnorm(P(mv,:) - R(mv,:),2,2),'omitnan');
end

function w = worldYawRate(quat, gyro)
% World-vertical component of the body angular rate (rad/s). For a flat tag
% this equals body gz; the quat rotation makes it tilt-robust.
w = 0;
if numel(quat) ~= 4 || numel(gyro) ~= 3, return; end
nq = norm(quat); if nq < 0.5, return; end
q = quat/nq; a=q(1); b=q(2); c=q(3); d=q(4);
R3 = [2*(b*d-c*a), 2*(c*d+b*a), 1-2*(b^2+c^2)];   % 3rd row of R(quat)
w = R3 * gyro(:);
end

function w = wander(P)
P = P(all(isfinite(P),2),:); if size(P,1) < 10, w = NaN; return; end
Pm = movmedian(P,11,1); w = 1000*max(vecnorm(Pm - median(Pm,1),2,2));
end

function e = rmse(P, tr)
P = P(all(isfinite(P),2),:); if isempty(P), e = NaN; return; end
e = 1000*sqrt(mean(vecnorm(P - tr,2,2).^2));
end
