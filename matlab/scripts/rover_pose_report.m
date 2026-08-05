function out = rover_pose_report(matFile, opts)
%ROVER_POSE_REPORT Full 6-DOF comparison for a rover run: x,y,z and roll,pitch,yaw.
%   out = rover_pose_report                    % newest run
%   out = rover_pose_report("...\rover_uwb_x.mat")
%
%   Compares every pose channel we can observe, against the AprilTag as truth:
%     x, y   : AprilTag (lever-arm corrected to the rig centre) vs the rigid
%              two-tag UWB estimate, after the fitted 2D frame transform.
%     z      : AprilTag only. The UWB solve is 2D (tag height is FIXED), so it
%              has no z to compare. The camera's own z is also noisy - see the
%              printed standard deviation - so it is shown as a caveat, not truth.
%     roll   : AprilTag vs tag-240 IMU vs the rover's own Teensy IMU.
%     pitch  : same three.
%     yaw    : AprilTag vs UWB rigid MHE (geometry + gyro) vs both IMUs.
%
%   UNITS WARNING (measured, not assumed): the AprilTag euler angles are in
%   RADIANS but the rover Teensy IMU reports DEGREES. This is converted here.
%
%   Every non-truth channel is compared after removing a CONSTANT offset
%   (circular mean of the difference), because each sensor defines "zero"
%   differently - the marker's printed orientation, the IMU's mounting, and the
%   UWB anchor frame are all rotated relative to each other. The offsets are
%   printed: a stable offset means the source tracks truth well.

arguments
    matFile string = ""
    opts.casadiPath string = ""
end

if matFile == ""
    d = dir(fullfile(dune.rootDir(), 'results', 'rover_runs', 'rover_uwb_*.mat'));
    assert(~isempty(d), 'no rover_uwb_*.mat found');
    [~, i] = max([d.datenum]); matFile = fullfile(d(i).folder, d(i).name);
end

% Reuse the trajectory analysis (rigid MHE + frame fit) without its figure.
R = rover_run_report(matFile, casadiPath=opts.casadiPath, plot=false);
S = load(matFile); data = S.data; meta = S.metadata; uwb = data.uwb;

%% ---- assemble every angle source on a common (radian) footing ----------
okT  = ~isnan(data.apriltag_pos(:,1));
tT   = data.t_posix(okT);
posT = data.apriltag_pos(okT, :);            % x y z  (metres)
rpyT = data.apriltag_euler(okT, :);          % radians (verified by range)

% rover Teensy IMU: DEGREES -> radians
okR  = ~isnan(data.imu_euler(:,1));
tR   = data.t_posix(okR);
rpyR = deg2rad(data.imu_euler(okR, :));

% tag-240 IMU quaternion -> roll/pitch/yaw (radians); only on 240's sweeps
okQ  = all(isfinite(uwb.quat), 2);
tQ   = uwb.t_posix(okQ);
rpyQ = zeros(sum(okQ), 3);
Q    = uwb.quat(okQ, :);
for i = 1:size(Q,1), rpyQ(i,:) = quatRPY(Q(i,:)); end

% UWB rigid MHE yaw (anchor frame) + the fitted frame rotation -> Kinect frame
yawU = nan(0,1); tU = nan(0,1);
if isfield(R,'Pose') && any(~isnan(R.Pose(:,3)))
    good = ~isnan(R.Pose(:,3));
    tU   = uwb.t_posix(good);
    dth  = 0;
    if isfield(R,'R'), dth = atan2(R.R(2,1), R.R(1,1)); end
    yawU = R.Pose(good,3) + dth;
end

fprintf('\nsources: AprilTag %d | rover IMU %d | tag240 IMU %d | UWB yaw %d\n', ...
        sum(okT), sum(okR), sum(okQ), numel(yawU));
fprintf('AprilTag z: mean %.3f m, std %.3f m  <-- camera depth is noisy; UWB has no z\n', ...
        mean(posT(:,3)), std(posT(:,3)));

%% ---- compare each channel on the truth timebase -------------------------
cmp = struct('name', {}, 'off', {}, 'rms', {}, 'n', {});
    function [dv, offs] = alignAngle(tSrc, aSrc, label)
        dv = nan(size(tT)); offs = NaN;
        if numel(tSrc) < 5, return; end
        ai = interp1(tSrc, unwrap(aSrc), tT, 'linear', NaN);
        m = isfinite(ai);
        if nnz(m) < 5, return; end
        dif = wrapToPi(ai(m) - rpyTsel(m));
        offs = atan2(mean(sin(dif)), mean(cos(dif)));   % circular mean offset
        dv(m) = wrapToPi(ai(m) - offs);
        r = wrapToPi(dv(m) - rpyTsel(m));
        cmp(end+1) = struct('name', label, 'off', rad2deg(offs), ...
                            'rms', rad2deg(sqrt(mean(r.^2))), 'n', nnz(m)); %#ok<AGROW>
    end

fprintf('\n=== angle agreement vs AprilTag (constant offset removed) ===\n');
fprintf('%-22s %10s %10s %6s\n', 'channel', 'offset(deg)', 'RMS(deg)', 'n');
A = struct();
for ch = 1:3
    chName = {'roll','pitch','yaw'}; rpyTsel = rpyT(:, ch);
    switch ch
        case 1
            [A.rollR, ~] = alignAngle(tR, rpyR(:,1), 'roll  rover-IMU');
            [A.rollQ, ~] = alignAngle(tQ, rpyQ(:,1), 'roll  tag240-IMU');
        case 2
            [A.pitchR, ~] = alignAngle(tR, rpyR(:,2), 'pitch rover-IMU');
            [A.pitchQ, ~] = alignAngle(tQ, rpyQ(:,2), 'pitch tag240-IMU');
        case 3
            [A.yawR, ~] = alignAngle(tR, rpyR(:,3), 'yaw   rover-IMU');
            [A.yawQ, ~] = alignAngle(tQ, rpyQ(:,3), 'yaw   tag240-IMU');
            [A.yawU, ~] = alignAngle(tU, yawU,      'yaw   UWB rigid MHE');
    end
end
for k = 1:numel(cmp)
    fprintf('%-22s %10.1f %10.2f %6d\n', cmp(k).name, cmp(k).off, cmp(k).rms, cmp(k).n);
end

%% ---- plot ---------------------------------------------------------------
t0 = data.t_posix(1); tp = tT - t0;
fig = figure('Position',[20 20 1500 950]);
tiledlayout(3,2,'TileSpacing','compact','Padding','compact');

% x
nexttile; hold on; grid on;
plot(tp, R.cT(:,1), 'b-', 'LineWidth',1.3);
if isfield(R,'Ealigned'), plot(R.tm-t0, R.Ealigned(:,1), 'r-'); end
ylabel('x (m)'); title('x  — truth (blue) vs UWB (red)'); xlabel('t (s)');

% y
nexttile; hold on; grid on;
plot(tp, R.cT(:,2), 'b-', 'LineWidth',1.3);
if isfield(R,'Ealigned'), plot(R.tm-t0, R.Ealigned(:,2), 'r-'); end
ylabel('y (m)'); title('y  — truth (blue) vs UWB (red)'); xlabel('t (s)');

% z
nexttile; hold on; grid on;
plot(tp, posT(:,3), 'b.-', 'MarkerSize',6);
yline(mean(posT(:,3)), 'k--');
ylabel('z (m)'); xlabel('t (s)');
title(sprintf('z — AprilTag only (std %.0f mm). UWB solve is 2D: no z.', ...
      1000*std(posT(:,3))));

% roll
nexttile; hold on; grid on;
plot(tp, rad2deg(rpyT(:,1)), 'b-', 'LineWidth',1.3);
plot(tp, rad2deg(A.rollR), 'r-'); plot(tp, rad2deg(A.rollQ), 'g-');
ylabel('roll (deg)'); xlabel('t (s)');
legend({'AprilTag','rover IMU','tag240 IMU'}, 'Location','best');
title('roll — offsets removed');

% pitch
nexttile; hold on; grid on;
plot(tp, rad2deg(rpyT(:,2)), 'b-', 'LineWidth',1.3);
plot(tp, rad2deg(A.pitchR), 'r-'); plot(tp, rad2deg(A.pitchQ), 'g-');
ylabel('pitch (deg)'); xlabel('t (s)');
legend({'AprilTag','rover IMU','tag240 IMU'}, 'Location','best');
title('pitch — offsets removed');

% yaw
nexttile; hold on; grid on;
plot(tp, rad2deg(rpyT(:,3)), 'b-', 'LineWidth',1.6);
plot(tp, rad2deg(A.yawU), 'm-', 'LineWidth',1.2);
plot(tp, rad2deg(A.yawR), 'r-'); plot(tp, rad2deg(A.yawQ), 'g-');
ylabel('yaw (deg)'); xlabel('t (s)');
legend({'AprilTag','UWB rigid MHE','rover IMU','tag240 IMU'}, 'Location','best');
title('yaw — the channel UWB actually estimates');

f = replace(char(matFile), '.mat', '_pose6dof.png');
saveas(fig, f);
fprintf('\nfigure: %s\n', f);
out = struct('file', matFile, 'cmp', cmp, 'A', A, 'rpyT', rpyT, 'posT', posT, ...
             'tT', tT, 'figFile', f);
end

function rpy = quatRPY(q)
rpy = [0 0 0];
nq = norm(q); if nq < 0.5 || numel(q) ~= 4, return; end
q = q/nq; w=q(1); x=q(2); y=q(3); z=q(4);
sp = 2*(w*y - z*x); sp = max(min(sp,1),-1);
rpy = [atan2(2*(w*x + y*z), 1 - 2*(x^2 + y^2)), asin(sp), ...
       atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))];
end
