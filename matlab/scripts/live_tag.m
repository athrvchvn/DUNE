function live_tag(port, opts)
%LIVE_TAG Live UWB tag position from the COM stream (step 3).
%   live_tag                 % auto-detect port (needs exactly one free COM)
%   live_tag("COM7")
%   live_tag("COM7", bias=true)   % ALSO subtract host-side anchor_bias.json
%
%   Streams RTLS sweeps from the tag over serial, solves each one with
%   dune.solveSweep (sentinel reject -> NLOS gap weights -> weighted LM) and
%   shows a live floor map: anchors, current tag position, recent trail, and
%   a rate / anchors-used / residual readout. Close the figure to stop.
%
%   Every sweep is logged as JSONL (dune.readSessionLog schema) to
%   matlab\logs\rtls_log_<stamp>.jsonl, and every raw serial line to
%   serial_raw_<stamp>.log, so any live run can be replayed later.
%
%   On the first sweep from a tag, GETMYDELAY is sent; the MYDELAY_ACK reply
%   (the tag's active NVS antenna delay, in ticks) is printed with all other
%   non-RTLS device lines as "[dev] ...".
%
%   Host-side bias defaults OFF: the boards' NVS antenna delays are the
%   source of truth (re-tuned in step 4); subtracting anchor_bias.json on
%   top would double-correct.
%
%   opts: bias (false), tagZ (0.22 m), trail (300 fixes), logDir (matlab\logs)

arguments
    port string = ""
    opts.bias (1,1) logical = false
    opts.powerCorr (1,1) logical = true   % DW1000 power-bias correction
    opts.nlos (1,1) logical = true        % soft NLOS gap weights (false =
                                          % rely on the MAD gate only; parked
                                          % tests show equal/slightly better)
    opts.hold (1,1) logical = true        % last-known-range hold: keep the
                                          % full anchor geometry when anchors
                                          % miss a sweep (no subset jumps)
    opts.ekf (1,1) logical = true         % FusionEkf smoothing on the display
    opts.estimator string = "ekf"         % "ekf" (default) or "mhe" = experimental
                                          % Moving Horizon Estimator (CasADi/IPOPT,
                                          % ~40% smoother in motion). The EKF still
                                          % runs for stillness + the pin's velocity
                                          % gate; MHE just provides the display fix.
    opts.horizon (1,1) double = 10        % MHE window length (sweeps)
    opts.model string = "cv"              % MHE motion model: "cv" (holonomic) or
                                          % "unicycle" (NON-HOLONOMIC car: gyro
                                          % heading + no sideways slip; for the RC
                                          % car / rover, not hand-carrying)
    opts.useImu (1,1) logical = true      % MHE only: gyro yaw rate -> turn/heading
                                          % (delta; helps real curves, no-op on
                                          % straight/translation motion)
    opts.casadiPath string = ""
    opts.mode string = "pos"              % "pos" = fix updates (robust);
                                          % "ranges" = tightly-coupled with
                                          % per-anchor bias MEMORY: no position
                                          % jump when the anchor subset changes
                                          % (dropouts/skips). Best while parked
                                          % or slow; may lag when carried fast.
    opts.tagZ (1,1) double = 0.24         % tag antenna height (matches truth clicks)
    opts.exclude double = []              % anchor ids to DROP before solving
                                          % (A/B a bad link, e.g. exclude=5;
                                          % ranging still happens on the tag,
                                          % raw log keeps the full sweep)
    opts.pin (1,1) logical = true         % output deadband while parked: hold
                                          % the reported position frozen while
                                          % parked and the fix stays within
                                          % pinRadius; sustained excursion or
                                          % motion releases it
    opts.pinRadius (1,1) double = 0.05    % m, deadband radius (the breathing
                                          % lives below ~5-7 cm)
    opts.pinResid (1,1) double = 0.06     % m, fallback pin trigger: solve fit
                                          % residual below this AND...
    opts.pinVel (1,1) double = 0.08       % ...EKF speed below this counts as
                                          % parked even when the IMU stays
                                          % jittery (residual+velocity, not
                                          % IMU stillness, gate the pin)
    opts.smooth (1,1) double = 0.6        % output EMA alpha for the MOVING
                                          % track (0<a<1, 1 = off). a=0.6 ~ 50%
                                          % less jerk for ~25 mm lag; lower =
                                          % smoother but laggier. Cosmetic:
                                          % the EKF state is untouched.
    opts.trail (1,1) double = 300
    opts.margin (1,1) double = 2.0   % plot margin around the anchors (m)
    opts.logDir string = ""
    opts.transport string = "serial" % "serial" (COM, the `port` arg) or "udp"
                                      % (WiFi: the tag already UDP-broadcasts; the
                                      % `port` arg is ignored, opts.udpPort binds).
                                      % One tag only — for two tags use live_dual.
    opts.udpPort (1,1) double = 4100  % UDP port the firmware streams to
end

A = dune.loadAnchors();
B = [];
if opts.bias, B = dune.loadAnchorBias(); end
RC = [];
if opts.powerCorr, RC = dune.loadRangeCorrection(); end

%% Log files
if opts.logDir == "", opts.logDir = fullfile(dune.rootDir(), 'logs'); end
if ~exist(opts.logDir, 'dir'), mkdir(opts.logDir); end
stamp = datestr(now, 'yyyymmdd_HHMMSS'); %#ok<TNOW1,DATST>
logFile = fullfile(opts.logDir, ['rtls_log_' stamp '.jsonl']);
fid = fopen(logFile, 'w');

% First line: the options this session actually ran under.
%
% WHY THIS MATTERS. sweepRecord writes x,y (the solve) and ex,ey (the pinned +
% smoothed output), which together are a complete oracle for re-verifying an
% estimator against a real session. But ex,ey depends on hold/pin/smooth/ekf/
% estimator/tagZ and on which anchors.json was loaded, and none of that used to
% be recorded anywhere. Replaying a log therefore meant GUESSING the settings:
% of 35 paired sessions in matlab/logs, only 6 could be reproduced exactly, and
% `hold` had to be recovered indirectly by noticing that a held sweep logs more
% anchors than the wire carried.
%
% One line at the top removes all of that guesswork for every future session.
% Readers that predate it are unaffected: it carries no tag_id, so
% dune.readSessionLog and rtls.session_log both treat it as a non-sweep record.
meta = struct('type', 'session_meta', 'schema', 'dune_session_meta_v1', ...
              'stamp', stamp, 'script', 'live_tag', ...
              't_host', posixtime(datetime('now', 'TimeZone', 'local')), ...
              'anchors_layout', A.layout, 'anchors_n', numel(A.ids), ...
              'opts', opts, 'rangeCorr', ~isempty(RC), 'anchorBias', opts.bias);
fprintf(fid, '%s\n', jsonencode(meta));

%% Transport (serial COM or WiFi UDP)
if opts.transport == "udp"
    ts = dune.TagUdp(opts.udpPort);
    ts.rawLogFid = fopen(fullfile(opts.logDir, ['udp_raw_' stamp '.log']), 'w');
    srcName = sprintf('UDP:%d', opts.udpPort);
else
    ts = dune.TagSerial(port);
    ts.rawLogFid = fopen(fullfile(opts.logDir, ['serial_raw_' stamp '.log']), 'w');
    srcName = sprintf('%s @ %d baud', ts.port, ts.baud);
end
cleanup = onCleanup(@() endSession(ts, fid, logFile));
ts.start();
fprintf('Listening on %s (%s, host bias %s, power corr %s)\n', ...
        srcName, A.layout, string(opts.bias), string(~isempty(RC)));
fprintf('Logging to %s\n', logFile);

%% Figure
fig = figure('Name', sprintf('DUNE live tag - %s', srcName), 'NumberTitle', 'off');
ax = axes(fig); hold(ax, 'on'); axis(ax, 'equal'); grid(ax, 'on');
% Per-anchor markers (scatter so each can recolour by freshness: green =
% answering, red = not seen for a while).
anchH = scatter(ax, A.pos(:, 1), A.pos(:, 2), 90, [0 0.6 0], '^', 'filled', ...
                'MarkerEdgeColor', 'k');
text(ax, A.pos(:, 1) + 0.05, A.pos(:, 2), compose('A%d', A.ids));
xlim(ax, [min(A.pos(:, 1)) - opts.margin, max(A.pos(:, 1)) + opts.margin]);
ylim(ax, [min(A.pos(:, 2)) - opts.margin, max(A.pos(:, 2)) + opts.margin]);
xlabel(ax, 'x (m)'); ylabel(ax, 'y (m)');
trailH = plot(ax, nan, nan, '.-', 'Color', [0.35 0.55 0.9], 'MarkerSize', 6);
rawH   = plot(ax, nan, nan, 'o', 'Color', [0.65 0.65 0.65], 'MarkerSize', 6);
dotH   = plot(ax, nan, nan, 'o', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r', ...
              'MarkerSize', 9);
ttl = title(ax, 'waiting for RTLS stream...');

%% Main loop
trail = nan(opts.trail, 2);
prevPos = [];
tRate = [];
nSweeps = 0; nSolved = 0; nRejected = 0;
queried = [];
ekf = dune.FusionEkf();
if opts.mode == "ranges"
    ekf.enableRangeBias(numel(A.ids));   % per-anchor bias memory
end
rh = dune.RangeHold();
mhe = [];
if opts.estimator == "mhe"
    dune.addCasadi(opts.casadiPath);
    mhe = dune.MheEstimator(A.pos);
    mhe.horizon = opts.horizon; mhe.tagZ = opts.tagZ; mhe.model = opts.model;
    mhe.build();
    fprintf('MHE estimator on (horizon %d, model %s, CasADi/IPOPT).\n', ...
            opts.horizon, opts.model);
end
tPrevEkf = NaN; tPrevMhe = NaN;
histA = nan(1, 8); histG = nan(1, 8);   % rolling stillness window (IMU ZUPT)
stillCnt = 0;                            % UWB-only stillness (no-IMU tags)
divergeStreak = 0;
pinPos = [NaN, NaN]; pinOut = 0;        % output-pin (deadband) state
pinned = false;
smoothPos = [NaN, NaN];                 % output EMA state (moving smoother)
missCount = zeros(1, numel(A.ids));     % consecutive sweeps each anchor absent

while ishandle(fig)
    for e = ts.drainEvents()
        fprintf('[dev] %s\n', e{1}.line);
    end
    for c = ts.drain()
        s = c{1};
        nSweeps = nSweeps + 1;
        % Per-anchor presence (from the RAW sweep, before RangeHold injects
        % held ranges): reset a counter when the anchor answers, else increment.
        present = ismember(A.ids, s.ids);
        missCount(present) = 0;
        missCount(~present) = missCount(~present) + 1;
        if ~ismember(s.tag, queried)
            queried(end+1) = s.tag; %#ok<AGROW>
            cmd = sprintf('GETMYDELAY,%d', s.tag);       % report NVS delay state
            try
                if isa(ts, 'dune.TagUdp')
                    ts.sendCmd(cmd, s.tag);
                else
                    ts.send(cmd);
                end
            catch
                % best-effort (cosmetic delay-source query); ignore failures
            end
        end
        % Stillness detection (drives range-hold trust and EKF stillMode)
        if ~isempty(s.imu) && s.imu.status >= 1
            histA = [histA(2:end), norm(s.imu.acc)];
            histG = [histG(2:end), norm(s.imu.gyro)];
            still = all(isfinite(histA)) && ...
                    max(histA) < 0.12 && max(histG) < 0.05;
        else
            % No IMU: infer stillness from the filter (calm innovations +
            % near-zero velocity for ~1.5 s; motion breaks it in a sweep).
            still = stillCnt >= 10;
        end

        % Last-known-range hold: full anchor geometry across dropouts
        nHeld = 0;
        if opts.hold, [s, nHeld] = rh.apply(s, still); end

        % Host-side anchor exclusion (A/B test of a suspect link). AFTER the
        % hold so RangeHold cannot re-inject the excluded anchor from memory.
        if ~isempty(opts.exclude)
            keep = ~ismember(s.ids, opts.exclude);
            s.ids = s.ids(keep); s.dist = s.dist(keep);
            s.rx = s.rx(keep); s.fp = s.fp(keep);
            if numel(s.qual) >= numel(keep), s.qual = s.qual(keep); end
            if isfield(s, 'wScale') && numel(s.wScale) >= numel(keep)
                s.wScale = s.wScale(keep);
            end
            if isfield(s, 'cfoPpm') && numel(s.cfoPpm) >= numel(keep)
                s.cfoPpm = s.cfoPpm(keep); s.tex = s.tex(keep);
            end
        end

        [p, info] = dune.solveSweep(s, A, bias=B, rangeCorr=RC, ...
                                    tagZ=opts.tagZ, x0=prevPos, ...
                                    useGapWeights=opts.nlos);
        nRejected = nRejected + nnz(info.rejected);

        % EKF smoothing (CV predict + gated position update + windowed ZUPT)
        pe = [NaN, NaN];
        if opts.ekf
            dt = 0;
            if isfinite(tPrevEkf), dt = min(max(s.thost - tPrevEkf, 0), 1); end
            tPrevEkf = s.thost;
            ekf.stillMode = still;   % still -> ZUPT + frozen process noise
            ekf.predict(dt, []);
            if opts.mode == "ranges"
                if ~ekf.initialized
                    if all(isfinite(p)), ekf.updatePosition(p); end
                else
                    ekf.updateRanges(A.pos, info.rangeCorr, info.w, ...
                                     opts.tagZ, 0.05);
                    solid = all(isfinite(p)) && nnz(info.used) >= 4 ...
                            && info.rmse < 0.10;
                    if solid && norm(ekf.pos - p) > 0.5
                        divergeStreak = divergeStreak + 1;
                    elseif solid
                        divergeStreak = 0;
                    end
                    if (ekf.consecReject >= ekf.maxConsecReject ...
                            || divergeStreak >= 5) && all(isfinite(p))
                        ekf.reinitFrom(p);
                        divergeStreak = 0;
                    end
                end
            elseif all(isfinite(p))
                ekf.updatePosition(p, adaptiveR(info, ekf.posSigma));
                if ekf.consecReject >= ekf.maxConsecReject
                    ekf.reinitFrom(p);
                end
            end
            if still && ekf.initialized
                ekf.updateZupt();
            end
            if isempty(s.imu) && ekf.initialized
                if norm(ekf.vel) < 0.08 && ekf.lastNis < 3
                    stillCnt = stillCnt + 1;
                else
                    stillCnt = 0;
                end
            end
            if ekf.initialized, pe = ekf.pos; end
        end
        % Experimental MHE: sliding-window smoother provides the display fix
        % (the EKF above still runs for stillness + the pin's velocity gate).
        if ~isempty(mhe)
            dtm = 0;
            if isfinite(tPrevMhe), dtm = min(max(s.thost - tPrevMhe, 0), 1); end
            tPrevMhe = s.thost;
            omega = 0;
            if opts.useImu && ~isempty(s.imu)
                omega = worldYawRate(s.imu.quat, s.imu.gyro);
            end
            [pm, ~, mi] = mhe.push(info.rangeCorr, info.w, max(dtm, 1e-3), p, still, omega);
            if ~mi.warmup && all(isfinite(pm))
                pe = pm;                 % MHE output feeds the pin/smoother/display
            elseif all(isfinite(p))
                pe = p;                  % warm-up: fall back to the raw fix
            end
        end
        % Output pin (deadband): while parked the true position is constant,
        % so any sub-radius movement of the estimate is breathing, not motion
        % - freeze the OUTPUT at the pin point. Purely an output-stage clamp:
        % the EKF/solver state is untouched (the A3 experiment showed that
        % absorbing breathing INSIDE the estimator corrupts it).
        po = p;
        if opts.ekf && all(isfinite(pe)), po = pe; end
        % Pin engages on IMU stillness OR, as a fallback for a jittery IMU, a
        % calm solve: low fit residual AND near-zero EKF speed. The fallback
        % lets a genuinely-parked tag pin even when the BNO085 will not settle
        % under the stillness thresholds; it releases the instant the solve
        % gets noisy or the tag actually moves.
        calm = isfinite(info.rmse) && info.rmse < opts.pinResid && ...
               opts.ekf && ekf.initialized && norm(ekf.vel) < opts.pinVel;
        pinnable = still || calm;
        pinned = false;
        if opts.pin && pinnable && all(isfinite(po))
            if any(~isfinite(pinPos)), pinPos = po; pinOut = 0; end
            if norm(po - pinPos) < opts.pinRadius
                pinOut = 0;
                po = pinPos; pinned = true;
            else
                pinOut = pinOut + 1;         % excursion beyond the deadband
                if pinOut >= 8               % sustained ~1.5 s -> re-pin there
                    pinPos = po; pinOut = 0; pinned = true;
                else
                    po = pinPos; pinned = true;   % brief excursion: hold
                end
            end
        else
            pinPos = [NaN, NaN]; pinOut = 0;
        end
        % Output smoother (moving only): a light EMA on the un-pinned track
        % kills the high-frequency fix jitter. Re-seeded to the pin point
        % while pinned so a release into motion starts without a lag jump.
        if pinned || ~all(isfinite(po))
            smoothPos = po;                          % track pin / hold on dropout
        elseif opts.smooth < 1 && all(isfinite(smoothPos))
            smoothPos = opts.smooth * po + (1 - opts.smooth) * smoothPos;
            po = smoothPos;
        else
            smoothPos = po;                          % first valid sample / off
        end
        fprintf(fid, '%s\n', jsonencode(dune.sweepRecord(s, p, info, A, po)));

        disp_ = po;
        % Per-anchor freshness: recolour the markers (green answering, orange
        % missing a few sweeps, red = dead) and name the dead ones in the title.
        acol = repmat([0 0.6 0], numel(A.ids), 1);
        acol(missCount >= 1, :) = repmat([0.95 0.6 0], nnz(missCount >= 1), 1);
        acol(missCount > 5, :)  = repmat([0.85 0 0], nnz(missCount > 5), 1);
        set(anchH, 'CData', acol);
        missStr = '';
        deadIds = A.ids(missCount > 5);
        if ~isempty(deadIds), missStr = ['  MISS:' sprintf(' A%d', deadIds)]; end
        if all(isfinite(p)), set(rawH, 'XData', p(1), 'YData', p(2)); end
        if all(isfinite(p))
            nSolved = nSolved + 1;
            prevPos = p;
            trail = [trail(2:end, :); disp_];
            tRate(end+1) = s.thost; %#ok<AGROW>
            tRate(tRate < s.thost - 10) = [];
            set(trailH, 'XData', trail(:, 1), 'YData', trail(:, 2));
            set(dotH, 'XData', disp_(1), 'YData', disp_(2));   % pinned/EKF value
            % Grow the view if the tag wanders outside it
            xl = xlim(ax); yl = ylim(ax);
            if p(1) < xl(1) || p(1) > xl(2) || p(2) < yl(1) || p(2) > yl(2)
                xlim(ax, [min(xl(1), p(1) - 0.5), max(xl(2), p(1) + 0.5)]);
                ylim(ax, [min(yl(1), p(2) - 0.5), max(yl(2), p(2) + 0.5)]);
            end
            hz = NaN;
            if numel(tRate) > 1, hz = (numel(tRate) - 1) / (tRate(end) - tRate(1)); end
            mode = '';
            if opts.ekf && all(isfinite(pe)), mode = ' EKF'; end
            if pinned, mode = [mode ' PIN']; end
            heldStr = '';
            if nHeld > 0, heldStr = sprintf(' (%d held)', nHeld); end
            ttl.String = sprintf(['tag %d%s   (%.2f, %.2f) m   %.1f Hz   ' ...
                                  '%d/%d anchors%s%s   resid %.0f mm   solved %d/%d'], ...
                s.tag, mode, disp_(1), disp_(2), hz, nnz(info.used), ...
                nnz(~isnan(info.range)), heldStr, missStr, 1000 * info.rmse, ...
                nSolved, nSweeps);
        else
            ttl.String = sprintf('tag %d   NO FIX (%d anchors usable)%s   solved %d/%d', ...
                s.tag, nnz(~isnan(info.range)), missStr, nSolved, nSweeps);
        end
    end
    drawnow limitrate
    pause(0.05);
end

fprintf('Figure closed: %d sweeps, %d solved, %d sentinel-rejected measurements.\n', ...
        nSweeps, nSolved, nRejected);
end

function endSession(ts, fid, logFile)
delete(ts);            % stops the callback, releases COM, closes raw log
fclose(fid);
fprintf('Session log: %s\n', logFile);
end

function R = adaptiveR(info, posSigma)
% LM covariance inflated by solver RMSE, mean NLOS gap, and a DOP proxy.
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

function w = worldYawRate(quat, gyro)
% World-vertical component of the body angular rate (rad/s) for the MHE
% coordinated-turn model. For a flat tag this equals body gz; the quat
% rotation keeps it correct under tilt. Uses only the rate (delta), not
% absolute heading, so an unvalidated orientation offset does not matter.
w = 0;
if numel(quat) ~= 4 || numel(gyro) ~= 3, return; end
nq = norm(quat); if nq < 0.5, return; end
q = quat / nq; a = q(1); b = q(2); c = q(3); d = q(4);
R3 = [2*(b*d - c*a), 2*(c*d + b*a), 1 - 2*(b^2 + c^2)];   % 3rd row of R(quat)
w = R3 * gyro(:);
end
