function live_rover(opts)
%LIVE_ROVER Live rigid two-tag rover pose (MHE only).
%   live_rover                        % both tags over UDP:4100
%   live_rover(transport="serial")    % wired tags, both on one COM port
%   live_rover(baseline=0.50)
%
%   Two firmware builds feed this. With UWB_COORD_RING both tags UDP-broadcast
%   their sweeps (HostLink UDP). With UWB_COORD_WIRE the IMU master grants the
%   other tag its slot over a UART cable and forwards its line, so both arrive
%   on one USB serial port. Either way a single link object ingests BOTH and
%   demuxes by tag_id. Every sweep - from either tag - is fed
%   as its own node to dune.MheRigid, which estimates ONE rigid body pose
%   [cx cy psi speed] with the non-holonomic unicycle model:
%     * inter-tag DISTANCE is exact (baked into the parameterisation),
%     * both tags stationary-or-moving together is automatic (one speed state),
%     * YAW is fused from the UWB geometry AND the gyro yaw rate,
%     * ROLL/PITCH are taken directly from the IMU (tag 240) and used for the
%       terrain geometry (projected baseline + per-tag antenna height).
%   No EKF anywhere - MHE only.
%
%   Tag roles: FRONT = 241 (no IMU), REAR = 240 (BNO085). Heading points
%   rear -> front. Set frontTag/rearTag if you swap them.
%
%   Display guard (the bug that broke the old live_dual_tag): the markers only
%   move on a fresh valid fix. If BOTH tags go quiet for coastLimit seconds the
%   display FREEZES and the title says COASTING - it never follows a
%   dead-reckoned estimate away from reality.

arguments
    opts.baseline (1,1) double = 0.527    % L, antenna centre-to-centre - MEASURE
    opts.frontTag (1,1) double = 241
    opts.rearTag (1,1) double = 240
    opts.transport string = "udp"         % "udp" | "serial" (wired master/slave
                                          %   tags: both arrive on one COM port)
    opts.udpPort (1,1) double = 4100
    opts.com string = ""                  % master tag's port; "" = auto-detect
    opts.tagZ (1,1) double = 0.24
    opts.horizon (1,1) double = 12
    opts.powerCorr (1,1) logical = true
    opts.nlos (1,1) logical = true
    opts.coastLimit (1,1) double = 2.0    % s without any fix -> freeze display
    opts.pin (1,1) logical = true         % freeze the DISPLAYED pose (centre AND
                                          % yaw) while the rig is still and inside
                                          % pinRadius. Cosmetic: the raw MHE pose
                                          % is still what gets logged.
    opts.pinRadius (1,1) double = 0.05    % m deadband (the breathing lives below
                                          % ~5-7 cm - see docs/phase_a_findings.md)
    opts.smooth (1,1) double = 0.6        % output EMA on the MOVING pose
                                          % (0<a<1, 1 = off); yaw smoothed circularly
    opts.trail (1,1) double = 400
    opts.margin (1,1) double = 2.0
    opts.logDir string = ""
    opts.casadiPath string = ""
end

dune.addCasadi(opts.casadiPath);
A = dune.loadAnchors();
RC = [];
if opts.powerCorr, RC = dune.loadRangeCorrection(); end

stamp = char(datetime('now', Format='yyyyMMdd_HHmmss'));
if opts.logDir == "", opts.logDir = fullfile(dune.rootDir(), 'logs'); end
if ~exist(opts.logDir, 'dir'), mkdir(opts.logDir); end
logFile = fullfile(opts.logDir, ['rover_log_' stamp '.jsonl']);
fid = fopen(logFile, 'w');

switch lower(opts.transport)
    case "udp"
        tu = dune.TagUdp(opts.udpPort);
        srcLabel = sprintf('UDP:%d', opts.udpPort);
    case "serial"
        % Wired master/slave firmware: the master forwards the slave's lines,
        % so both tag ids arrive on this one port and the demux is unchanged.
        if strlength(opts.com) == 0, tu = dune.TagSerial();
        else,                        tu = dune.TagSerial(opts.com);
        end
        srcLabel = sprintf('SERIAL:%s', tu.port);
    otherwise
        error('live_rover:badTransport', ...
              'transport must be "udp" or "serial", got "%s"', opts.transport);
end
tu.rawLogFid = fopen(fullfile(opts.logDir, ['udp_raw_' stamp '.log']), 'w');
cleanup = onCleanup(@() endSession(tu, fid, logFile));
tu.start();
fprintf('live_rover: %s | front=%d rear=%d | L=%.3f m | %s\n', ...
        srcLabel, opts.frontTag, opts.rearTag, opts.baseline, A.layout);
fprintf('Logging to %s\n', logFile);

mr = dune.MheRigid(A.pos);
mr.baseline = opts.baseline; mr.horizon = opts.horizon; mr.tagZ = opts.tagZ;
mr.build();
fprintf('MheRigid built (horizon %d, CasADi/IPOPT).\n', opts.horizon);

%% Figure
fig = figure('Name', sprintf('DUNE rover - %s', srcLabel), 'NumberTitle', 'off');
ax = axes(fig); hold(ax, 'on'); axis(ax, 'equal'); grid(ax, 'on');
anchH = scatter(ax, A.pos(:,1), A.pos(:,2), 90, [0 0.6 0], '^', 'filled', ...
                'MarkerEdgeColor', 'k');
text(ax, A.pos(:,1) + 0.05, A.pos(:,2), compose('A%d', A.ids));
xlim(ax, [min(A.pos(:,1)) - opts.margin, max(A.pos(:,1)) + opts.margin]);
ylim(ax, [min(A.pos(:,2)) - opts.margin, max(A.pos(:,2)) + opts.margin]);
xlabel(ax, 'x (m)'); ylabel(ax, 'y (m)');
trailH = plot(ax, nan, nan, '-', 'Color', [0.6 0.4 0.9], 'LineWidth', 1.2);
ghostH = plot(ax, nan, nan, 'o', 'Color', [0.75 0.75 0.75], 'MarkerSize', 5);
rigH   = plot(ax, nan, nan, '-', 'Color', [0.2 0.2 0.2], 'LineWidth', 2);
frontH = plot(ax, nan, nan, 'o', 'MarkerFaceColor', [0.1 0.4 0.9], ...
              'MarkerEdgeColor', 'k', 'MarkerSize', 9);
rearH  = plot(ax, nan, nan, 'o', 'MarkerFaceColor', [0.9 0.2 0.2], ...
              'MarkerEdgeColor', 'k', 'MarkerSize', 9);
ctrH   = plot(ax, nan, nan, 'p', 'MarkerFaceColor', [0.6 0.2 0.8], ...
              'MarkerEdgeColor', 'k', 'MarkerSize', 12);
hdgH   = quiver(ax, nan, nan, nan, nan, 0, 'Color', [0.6 0.2 0.8], 'LineWidth', 1.6, ...
                'MaxHeadSize', 2);
ttl = title(ax, 'waiting for UDP stream from both tags...');

%% State
trail = nan(opts.trail, 2);
missCount = zeros(1, numel(A.ids));
rawFix = containers.Map('KeyType','double','ValueType','any');   % tag -> [x y]
rawT   = containers.Map('KeyType','double','ValueType','any');   % tag -> thost
omegaHold = 0; pitchHold = 0; rollHold = 0; imuYaw = NaN;
histA = nan(1,8); histG = nan(1,8); still = false;
tPrev = NaN; tLastFix = NaN;
nSweeps = 0; nPose = 0; tRate = [];
pose = [NaN NaN NaN NaN];
pinPose = [NaN NaN NaN]; pinOut = 0; pinned = false;   % output-pin state
smoothC = [NaN NaN]; smoothPsi = NaN;                  % output EMA state

while ishandle(fig)
    for e = tu.drainEvents()
        fprintf('[dev %s] %s\n', e{1}.srcIp, e{1}.line);
    end
    for c = tu.drain()
        s = c{1};
        if ~ismember(s.tag, [opts.frontTag, opts.rearTag]), continue; end
        nSweeps = nSweeps + 1;
        isFront = (s.tag == opts.frontTag);

        % ---- per-anchor freshness (either tag counts as "seen") ----------
        present = ismember(A.ids, s.ids);
        missCount(present) = 0;
        missCount(~present) = missCount(~present) + 1;

        % ---- IMU (rear tag only): roll/pitch direct, gyro -> yaw rate -----
        if ~isempty(s.imu) && s.imu.status >= 1
            [rollHold, pitchHold, imuYaw] = quatRPY(s.imu.quat);
            omegaHold = worldYawRate(s.imu.quat, s.imu.gyro);
            histA = [histA(2:end), norm(s.imu.acc)];
            histG = [histG(2:end), norm(s.imu.gyro)];
            still = all(isfinite(histA)) && max(histA) < 0.12 && max(histG) < 0.05;
        end

        % ---- standalone raw fix for this tag (seed/guard/ghost) -----------
        sgn = -1; if isFront, sgn = 1; end
        tz = opts.tagZ + sgn * (opts.baseline/2) * sin(pitchHold);
        [p, info] = dune.solveSweep(s, A, rangeCorr=RC, tagZ=tz, ...
                                    useGapWeights=opts.nlos);
        if all(isfinite(p))
            rawFix(s.tag) = p; rawT(s.tag) = s.thost; tLastFix = s.thost;
        end

        % ---- MHE push ----------------------------------------------------
        dt = 0;
        if isfinite(tPrev), dt = min(max(s.thost - tPrev, 0), 1); end
        tPrev = s.thost;
        [poseNew, mi] = mr.push(info.rangeCorr, info.w, isFront, max(dt,1e-3), ...
                                p, still, omegaHold, pitchHold);
        if ~mi.warmup && all(isfinite(poseNew))
            pose = poseNew; nPose = nPose + 1;
            tRate(end+1) = s.thost; %#ok<AGROW>
            tRate(tRate < s.thost - 10) = [];
        end

        % ---- Output pin + smoother (COSMETIC: the raw MHE pose is logged) --
        % Parked, the true pose is constant, so sub-radius movement is the
        % known per-anchor breathing (docs/phase_a_findings.md), not motion:
        % freeze the DISPLAYED centre AND yaw. Moving, a light circular EMA
        % takes the high-frequency jitter off. The estimator is untouched.
        poseOut = pose;
        pinned = false;
        if all(isfinite(pose))
            if opts.pin && still
                if any(~isfinite(pinPose)), pinPose = pose(1:3); pinOut = 0; end
                if norm(pose(1:2) - pinPose(1:2)) >= opts.pinRadius
                    pinOut = pinOut + 1;
                    if pinOut >= 8, pinPose = pose(1:3); pinOut = 0; end
                else
                    pinOut = 0;
                end
                poseOut(1:3) = pinPose; pinned = true;
                smoothC = poseOut(1:2); smoothPsi = poseOut(3);   % reseed EMA
            else
                pinPose = [NaN NaN NaN]; pinOut = 0;
                if opts.smooth < 1 && all(isfinite(smoothC)) && isfinite(smoothPsi)
                    smoothC = opts.smooth*pose(1:2) + (1-opts.smooth)*smoothC;
                    sv = opts.smooth*[cos(pose(3)) sin(pose(3))] + ...
                         (1-opts.smooth)*[cos(smoothPsi) sin(smoothPsi)];
                    smoothPsi = atan2(sv(2), sv(1));      % circular EMA for yaw
                    poseOut(1:2) = smoothC; poseOut(3) = smoothPsi;
                else
                    smoothC = pose(1:2); smoothPsi = pose(3);
                end
            end
        end

        % ---- independent baseline yaw (raw fixes), for comparison --------
        yawRaw = NaN;
        if rawFix.isKey(opts.frontTag) && rawFix.isKey(opts.rearTag)
            d = rawFix(opts.frontTag) - rawFix(opts.rearTag);
            yawRaw = atan2(d(2), d(1));
        end

        % ---- log ---------------------------------------------------------
        rec = struct('t_host', s.thost, 'tag_id', s.tag, 'isFront', isFront, ...
                     'nUsed', nnz(info.used), 'still', still);
        if all(isfinite(p)), rec.x = round(p(1),4); rec.y = round(p(2),4); end
        if all(isfinite(pose))
            rec.cx = round(pose(1),4); rec.cy = round(pose(2),4);
            rec.psi = round(pose(3),4); rec.speed = round(pose(4),4);
        end
        rec.roll = round(rollHold,4); rec.pitch = round(pitchHold,4);
        rec.pinned = pinned;
        if isfinite(imuYaw), rec.imuYaw = round(imuYaw,4); end
        if isfinite(yawRaw), rec.yawRaw = round(yawRaw,4); end
        fprintf(fid, '%s\n', jsonencode(rec));

        % ---- display (guarded: never follow a coast) ---------------------
        acol = repmat([0 0.6 0], numel(A.ids), 1);
        acol(missCount >= 1, :) = repmat([0.95 0.6 0], nnz(missCount >= 1), 1);
        acol(missCount > 5, :)  = repmat([0.85 0 0], nnz(missCount > 5), 1);
        set(anchH, 'CData', acol);
        missStr = '';
        deadIds = A.ids(missCount > 5);
        if ~isempty(deadIds), missStr = ['  MISS:' sprintf(' A%d', deadIds)]; end

        coasting = isfinite(tLastFix) && (s.thost - tLastFix) > opts.coastLimit;
        gx = []; gy = [];
        for k = [opts.frontTag, opts.rearTag]
            if rawFix.isKey(k), q = rawFix(k); gx(end+1) = q(1); gy(end+1) = q(2); end %#ok<AGROW>
        end
        set(ghostH, 'XData', gx, 'YData', gy);

        if all(isfinite(poseOut)) && ~coasting
            [pf, prr] = mr.tagPositions(poseOut, pitchHold);
            set(frontH, 'XData', pf(1), 'YData', pf(2));
            set(rearH,  'XData', prr(1), 'YData', prr(2));
            set(rigH,   'XData', [prr(1) pf(1)], 'YData', [prr(2) pf(2)]);
            set(ctrH,   'XData', poseOut(1), 'YData', poseOut(2));
            set(hdgH, 'XData', poseOut(1), 'YData', poseOut(2), ...
                      'UData', 0.4*cos(poseOut(3)), 'VData', 0.4*sin(poseOut(3)));
            trail = [trail(2:end,:); poseOut(1:2)];
            set(trailH, 'XData', trail(:,1), 'YData', trail(:,2));
            xl = xlim(ax); yl = ylim(ax);
            if poseOut(1) < xl(1) || poseOut(1) > xl(2) || ...
               poseOut(2) < yl(1) || poseOut(2) > yl(2)
                xlim(ax, [min(xl(1), poseOut(1)-0.5), max(xl(2), poseOut(1)+0.5)]);
                ylim(ax, [min(yl(1), poseOut(2)-0.5), max(yl(2), poseOut(2)+0.5)]);
            end
        end

        hz = NaN;
        if numel(tRate) > 1, hz = (numel(tRate)-1)/(tRate(end)-tRate(1)); end
        yawStr = '';
        if all(isfinite(poseOut))
            yawStr = sprintf('yaw %+6.1f', rad2deg(wrapToPi(poseOut(3))));
            if isfinite(yawRaw)
                yawStr = sprintf('%s (raw %+6.1f)', yawStr, rad2deg(wrapToPi(yawRaw)));
            end
        end
        stateStr = 'MHE';
        if coasting, stateStr = 'COASTING - display frozen'; end
        if still, stateStr = [stateStr ' STILL']; end
        if pinned, stateStr = [stateStr ' PIN']; end
        if all(isfinite(poseOut))
            ttl.String = sprintf(['rover %s  (%.2f, %.2f) m  %s deg  v %.2f m/s  ' ...
                'roll %+.1f pitch %+.1f  %.1f Hz  %d/%d anch%s  poses %d/%d'], ...
                stateStr, poseOut(1), poseOut(2), yawStr, poseOut(4), ...
                rad2deg(rollHold), rad2deg(pitchHold), hz, ...
                nnz(info.used), nnz(~isnan(info.range)), missStr, nPose, nSweeps);
        else
            ttl.String = sprintf('rover %s  filling window (%d/%d)%s  sweeps %d', ...
                stateStr, numel(mr.buf), opts.horizon, missStr, nSweeps);
        end
    end
    drawnow limitrate
    pause(0.05);
end

fprintf('Closed: %d sweeps, %d poses, %d guard trips.\n', nSweeps, nPose, mr.nJumps);
end

%% ── helpers ──────────────────────────────────────────────────────────────
function endSession(tu, fid, logFile)
delete(tu);
fclose(fid);
fprintf('Session log: %s\n', logFile);
end

function [roll, pitch, yaw] = quatRPY(q)
% BNO085 quaternion [qw qx qy qz] -> roll/pitch/yaw (rad).
roll = 0; pitch = 0; yaw = NaN;
if numel(q) ~= 4, return; end
nq = norm(q); if nq < 0.5, return; end
q = q/nq; w = q(1); x = q(2); y = q(3); z = q(4);
roll  = atan2(2*(w*x + y*z), 1 - 2*(x^2 + y^2));
sp = 2*(w*y - z*x); sp = max(min(sp,1),-1);
pitch = asin(sp);
yaw   = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2));
end

function w = worldYawRate(quat, gyro)
% World-vertical component of the body angular rate (rad/s). Delta-only: the
% absolute (rotated) heading is never trusted, just the turn rate.
w = 0;
if numel(quat) ~= 4 || numel(gyro) ~= 3, return; end
nq = norm(quat); if nq < 0.5, return; end
q = quat/nq; a = q(1); b = q(2); c = q(3); d = q(4);
R3 = [2*(b*d - c*a), 2*(c*d + b*a), 1 - 2*(b^2 + c^2)];
w = R3 * gyro(:);
end
