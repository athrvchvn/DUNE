function out = rover_run_report(matFile, opts)
%ROVER_RUN_REPORT Analyse a rover_teleop_uwb capture: UWB vs AprilTag truth.
%   out = rover_run_report                     % newest file in results/rover_runs
%   out = rover_run_report("...\rover_uwb_x.mat")
%
%   Runs the estimator offline over the logged sweeps (per-tag raw fixes and
%   the rigid two-tag MHE), lines them up against the AprilTag truth on the
%   shared POSIX clock, fits the 2D rigid transform between the Kinect world
%   and the UWB anchor world (they are NOT assumed equal), and reports the
%   moving accuracy. Also plots the data-quality diagnostics that decide
%   whether a run is usable at all.

arguments
    matFile string = ""
    opts.casadiPath string = ""
    opts.runMhe (1,1) logical = true
    opts.plot (1,1) logical = true
    opts.anchorsFile string = ""   % override anchors.json (e.g. a self-survey
                                   % geometry) to test a Kinect-free constellation
    opts.truthSpeedGate (1,1) double = 4    % reject truth samples implying more
                                   % than this multiple of the rover's v_max
                                   % (set Inf to keep every truth sample)
end

dune.addCasadi(opts.casadiPath);
if matFile == ""
    d = dir(fullfile(dune.rootDir(), 'results', 'rover_runs', 'rover_uwb_*.mat'));
    assert(~isempty(d), 'no rover_uwb_*.mat found');
    [~, i] = max([d.datenum]);
    matFile = fullfile(d(i).folder, d(i).name);
end
fprintf('Loading %s\n', matFile);
S = load(matFile);
data = S.data; meta = S.metadata;
uwb = data.uwb;
if opts.anchorsFile == ""
    A = dune.loadAnchors();
else
    A = dune.loadAnchors(opts.anchorsFile);
    fprintf('ANCHORS OVERRIDE: %s\n', opts.anchorsFile);
end
RC = dune.loadRangeCorrection();

L = meta.uwb.baseline_m;
offBody = meta.uwb.apriltag_offset_body(:)';   % [forward, left] in body axes
frontTag = meta.uwb.frontTag; rearTag = meta.uwb.rearTag;
nS = numel(uwb.t_posix);
fprintf('%d UWB sweeps | %d rover samples | L=%.3f m | AprilTag offset [%.4f %.4f]\n', ...
        nS, numel(data.time), L, offBody);

%% ---- per-sweep raw fix (each tag solved independently) ------------------
Praw = nan(nS, 2); nAnch = zeros(nS, 1);
prev = [];
for i = 1:nS
    have = find(~isnan(uwb.R(i, :)));
    nAnch(i) = numel(have);
    if numel(have) < 3, continue; end
    sw = struct('tag', uwb.tag(i), 'ids', uwb.anchorIds(have), ...
                'dist', uwb.R(i, have), 'rx', uwb.RX(i, have), 'fp', uwb.FP(i, have));
    p = dune.solveSweep(sw, A, rangeCorr=RC, tagZ=0.24, x0=prev);
    Praw(i, :) = p;
    if all(isfinite(p)), prev = p; end
end
fprintf('raw per-sweep fixes: %d/%d solved (%.0f%%)\n', ...
        sum(~isnan(Praw(:,1))), nS, 100*mean(~isnan(Praw(:,1))));

%% ---- rigid two-tag MHE over the same sweeps -----------------------------
Pose = nan(nS, 4);
if opts.runMhe
    mr = dune.MheRigid(A.pos);
    mr.baseline = L; mr.horizon = 12; mr.tagZ = 0.24;
    mr.build();
    % still / omega / pitch per sweep
    Vc = interp1(data.t_posix, data.V_cmd, uwb.t_posix, 'previous', 'extrap');
    Wc = interp1(data.t_posix, data.omega_cmd, uwb.t_posix, 'previous', 'extrap');
    stillAll = abs(Vc) < 1e-3 & abs(Wc) < 1e-3;
    omHold = 0; pitchHold = 0;
    tPrev = NaN;
    for i = 1:nS
        if all(isfinite(uwb.quat(i,:)))
            [~, pitchHold] = quatRP(uwb.quat(i,:));
            omHold = worldYawRate(uwb.quat(i,:), uwb.gyro(i,:));
        end
        have = find(~isnan(uwb.R(i, :)));
        z = nan(numel(A.ids),1); w = zeros(numel(A.ids),1);
        if numel(have) >= 3
            sw = struct('tag', uwb.tag(i), 'ids', uwb.anchorIds(have), ...
                'dist', uwb.R(i,have), 'rx', uwb.RX(i,have), 'fp', uwb.FP(i,have));
            [~, info] = dune.solveSweep(sw, A, rangeCorr=RC, tagZ=0.24);
            z = info.rangeCorr; w = info.w;
        end
        dt = 0.15; if isfinite(tPrev), dt = min(max(uwb.t_posix(i)-tPrev,1e-3),1); end
        tPrev = uwb.t_posix(i);
        [pose, ~] = mr.push(z, w, uwb.tag(i)==frontTag, dt, Praw(i,:), ...
                            stillAll(i), omHold, pitchHold);
        Pose(i,:) = pose;
    end
    fprintf('rigid MHE: %d/%d poses (%.0f%%), guard trips %d\n', ...
        sum(~isnan(Pose(:,1))), nS, 100*mean(~isnan(Pose(:,1))), mr.nJumps);
end

%% ---- AprilTag truth, lever-arm corrected --------------------------------
okT = ~isnan(data.apriltag_pos(:,1));
tT = data.t_posix(okT);
xyT = data.apriltag_pos(okT, 1:2);
yawT = data.apriltag_euler(okT, 3);
% centre of the UWB rig implied by truth: rotate the body-frame lever arm out
cT = xyT - [cos(yawT).*offBody(1) - sin(yawT).*offBody(2), ...
            sin(yawT).*offBody(1) + cos(yawT).*offBody(2)];
fprintf('AprilTag truth samples: %d (%.0f%% of rover samples)\n', ...
        sum(okT), 100*mean(okT));

% Kinematic plausibility gate on the truth itself. The Kinect logger drops out
% for ~10 s at a time; the LAST sample before each outage carries a pre-outage
% timestamp but a post-outage pose, so it reads as the rover teleporting
% ~0.8 m in one 0.37 s frame. The rover is commanded at v_max = 0.1 m/s and
% physically cannot do that, so those samples are truth artefacts, not UWB
% error -- and left in they dominate the RMSE. Reject any truth sample whose
% step from its predecessor (same contiguous run, dt < 1 s) implies a speed
% above truthSpeedGate * v_max. Deliberately loose: the lever-arm term adds
% only ~0.02 m/s at omega_max, so nothing real gets near 4x.
if isfinite(opts.truthSpeedGate) && numel(tT) > 2
    vLim = opts.truthSpeedGate * meta.v_max;
    dtT = diff(tT(:));
    stepT = vecnorm(diff(cT, 1, 1), 2, 2);
    bad = [false; dtT < 1 & stepT ./ dtT > vLim];
    if any(bad)
        fprintf(['truth speed gate (%.2f m/s): dropping %d of %d samples ' ...
                 '(implied speed up to %.2f m/s)\n'], vLim, sum(bad), ...
                numel(bad), max(stepT(bad(2:end)) ./ dtT(bad(2:end))));
        fprintf('  rejected at t = %s s\n', ...
                join(compose('%.2f', tT(bad) - data.t_posix(1)), ', '));
        tT = tT(~bad); cT = cT(~bad, :);
        okT(find(okT, numel(bad))) = ~bad;   % keep the availability plot honest
    end
end
nTruthRejected = sum(~isnan(data.apriltag_pos(:,1))) - numel(tT);

%% ---- align UWB estimate to truth times, fit 2D rigid transform ----------
out = struct('file', matFile, 'Praw', Praw, 'Pose', Pose, 'cT', cT, 'tT', tT, ...
             'truthRejected', nTruthRejected);
src = Pose(:,1:2); srcT = uwb.t_posix;
if all(isnan(src(:,1))), src = Praw; end
good = ~isnan(src(:,1));
if sum(good) > 20 && numel(tT) > 20
    Ei = interp1(srcT(good), src(good,:), tT, 'linear', NaN);
    m = all(isfinite(Ei),2) & all(isfinite(cT),2);
    fprintf('overlapping samples for alignment: %d\n', sum(m));
    if sum(m) > 20
        [Rr, tr, rms] = fitRigid2D(Ei(m,:), cT(m,:));
        Ea = (Rr * Ei(m,:)')' + tr;      % UWB -> Kinect frame
        err = vecnorm(Ea - cT(m,:), 2, 2);
        fprintf('\n=== UWB vs AprilTag truth (after 2D rigid alignment) ===\n');
        fprintf('  rotation %.1f deg, translation [%.2f %.2f] m\n', ...
                rad2deg(atan2(Rr(2,1),Rr(1,1))), tr);
        fprintf('  RMSE %.0f mm | median %.0f mm | p95 %.0f mm | max %.0f mm\n', ...
                1000*rms, 1000*median(err), 1000*prctile(err,95), 1000*max(err));
        % Same transform applied to EVERY UWB pose, not just the truth-time
        % samples, so panel 3 can show the full-rate track. Without this the
        % aligned panel draws only the ~1 Hz decimation and looks like a
        % different curve from the 6 Hz track in panel 1.
        out.PoseAligned = nan(size(src));
        out.PoseAligned(good,:) = (Rr * src(good,:)')' + tr;
        pathU = sum(vecnorm(diff(Ea), 2, 2));
        pathT = sum(vecnorm(diff(cT(m,:)), 2, 2));
        fprintf('  path length: UWB %.2f m vs truth %.2f m (%.2fx -> jitter)\n', ...
                pathU, pathT, pathU / max(pathT, 1e-6));
        out.err = err; out.Ealigned = Ea; out.cTm = cT(m,:); out.tm = tT(m);
        out.R = Rr; out.t = tr; out.pathRatio = pathU / max(pathT, 1e-6);
    end
end

%% ---- plots -------------------------------------------------------------
if ~opts.plot, return; end
TRAJ_LIM = [-3 3 -2 2];        % common x/y limits for all trajectory panels
fig = figure('Position',[30 30 1500 900]);
tiledlayout(2,3,'TileSpacing','compact','Padding','compact');

% 1: UWB world - anchors, raw fixes, MHE
nexttile; hold on; grid on; axis equal;
plot(A.pos(:,1), A.pos(:,2), 'k^','MarkerFaceColor','y','MarkerSize',9);
text(A.pos(:,1)+0.05, A.pos(:,2), compose('A%d', A.ids), 'Clipping','on');
plot(Praw(:,1), Praw(:,2), '.', 'Color',[.75 .75 .75], 'MarkerSize',4);
if any(~isnan(Pose(:,1)))
    plot(Pose(:,1), Pose(:,2), '-', 'Color',[.85 .2 .2], 'LineWidth',1.2);
end
title('UWB frame: raw fixes (grey) + rigid MHE (red)'); xlabel('x (m)'); ylabel('y (m)');
lockTraj(TRAJ_LIM);

% 2: AprilTag truth in its own frame
nexttile; hold on; grid on; axis equal;
plot(cT(:,1), cT(:,2), 'b.-', 'MarkerSize',5);
title(sprintf('AprilTag truth (Kinect frame), %d samples (%d gated)', ...
              size(cT,1), nTruthRejected));
xlabel('x (m)'); ylabel('y (m)'); lockTraj(TRAJ_LIM);

% 3: overlay after alignment
nexttile; hold on; grid on; axis equal;
if isfield(out,'Ealigned')
    plot(out.PoseAligned(:,1), out.PoseAligned(:,2), '-', ...
         'Color',[1 .65 .65], 'LineWidth',0.8);
    plot(out.cTm(:,1), out.cTm(:,2), 'b-', 'LineWidth',1.4);
    plot(out.Ealigned(:,1), out.Ealigned(:,2), 'r-', 'LineWidth',1.1);
    legend({'UWB full rate','truth','UWB @ truth times'}, 'Location','best');
    title(sprintf('aligned: RMSE %.0f mm, path %.1fx truth', ...
                  1000*sqrt(mean(out.err.^2)), out.pathRatio));
else
    title('not enough overlap to align');
end
xlabel('x (m)'); ylabel('y (m)'); lockTraj(TRAJ_LIM);

% 4: commands
nexttile; hold on; grid on;
plot(data.time, data.V_cmd, 'b-'); plot(data.time, data.omega_cmd, 'r-');
legend({'V cmd (m/s)','\omega cmd (rad/s)'},'Location','best');
xlabel('t (s)'); title('commanded velocity');

% 5: UWB health
nexttile; hold on; grid on;
tRel = uwb.t_posix - uwb.t_posix(1);
plot(tRel, nAnch, '.', 'MarkerSize',4);
isF = uwb.tag == frontTag;
plot(tRel(isF), 5.4*ones(sum(isF),1), '.', 'Color',[.1 .4 .9], 'MarkerSize',3);
plot(tRel(~isF), 5.7*ones(sum(~isF),1), '.', 'Color',[.9 .2 .2], 'MarkerSize',3);
ylim([0 6]); xlabel('t (s)'); ylabel('anchors / sweep');
title(sprintf('UWB health: front %d (blue), rear %d (red)', sum(isF), sum(~isF)));

% 6: truth availability + error vs time
nexttile; hold on; grid on;
plot(data.time, double(okT), 'k.', 'MarkerSize',4);
if isfield(out,'err')
    yyaxis right
    plot(out.tm - data.t_posix(1), 1000*out.err, 'r-');
    ylabel('|error| (mm)');
end
yyaxis left; ylabel('AprilTag valid'); ylim([-0.1 1.1]);
xlabel('t (s)'); title('truth availability (black) + error (red)');

f = replace(char(matFile), '.mat', '_report.png');
saveas(fig, f);
fprintf('\nfigure: %s\n', f);
out.figFile = f;
end

%% ── helpers ─────────────────────────────────────────────────────────────
function lockTraj(lim)
%LOCKTRAJ Pin equal-aspect limits so a window resize cannot re-expand them.
%   `axis equal` on its own only fixes the data aspect ratio - the limits stay
%   free to grow when the axes box is reshaped, which is why a resized figure
%   showed different x-ranges per tile. Matching the plot box to the data range
%   satisfies both constraints and holds the limits put.
axis(lim);
daspect([1 1 1]);
pbaspect([diff(lim(1:2)) diff(lim(3:4)) 1]);
end

function [R, t, rms] = fitRigid2D(P, Q)
% Least-squares 2D rigid transform (no scale) mapping P -> Q (Umeyama).
mp = mean(P,1); mq = mean(Q,1);
Pc = P - mp; Qc = Q - mq;
H = Pc' * Qc;
[U,~,V] = svd(H);
D = diag([1, sign(det(V*U'))]);
R = V * D * U';
t = mq - (R*mp')';
E = (R*P')' + t - Q;
rms = sqrt(mean(sum(E.^2,2)));
end

function [roll, pitch] = quatRP(q)
roll = 0; pitch = 0;
if numel(q) ~= 4, return; end
nq = norm(q); if nq < 0.5, return; end
q = q/nq; w=q(1); x=q(2); y=q(3); z=q(4);
roll = atan2(2*(w*x + y*z), 1 - 2*(x^2 + y^2));
sp = 2*(w*y - z*x); sp = max(min(sp,1),-1); pitch = asin(sp);
end

function w = worldYawRate(quat, gyro)
w = 0;
if numel(quat) ~= 4 || numel(gyro) ~= 3, return; end
nq = norm(quat); if nq < 0.5, return; end
q = quat/nq; a=q(1); b=q(2); c=q(3); d=q(4);
R3 = [2*(b*d - c*a), 2*(c*d + b*a), 1 - 2*(b^2 + c^2)];
w = R3 * gyro(:);
end
