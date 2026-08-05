function out = motion_check(logFile, opts)
%MOTION_CHECK Assess moving-tag behaviour from a live_tag JSONL log.
%   out = motion_check("D:\UWB_3\matlab\logs\rtls_log_xxx.jsonl")
%
%   No Kinect truth required - this is the poor-man's motion check: it
%   segments the session into STILL and MOVING stretches, and reports the
%   metrics that decide whether the moving pipeline is healthy before we
%   build the Kinect moving-truth recorder (7.4):
%     - EKF-vs-raw scatter while still  (pin/ZUPT working?)
%     - per-move displacement           (does a ~50 cm move read ~50 cm?)
%     - EKF lag behind raw during motion (over-smoothed / laggy?)
%     - path smoothness (jerk) of the EKF track vs raw
%     - pin coverage: pinned while still, released while moving
%
%   The JSONL carries raw fix (x,y) and the pinned/EKF output (ex,ey).
%   Produces a figure (raw ghost + EKF track, still/move shading) and prints
%   a summary. Mark your known moves by pausing ~2 s at each waypoint.

arguments
    logFile (1,1) string
    opts.stillSpeed (1,1) double = 0.06   % m/s below which a smoothed sample is "still"
    opts.minStillS  (1,1) double = 1.5    % s to count as a genuine dwell
    opts.plot (1,1) logical = true
end

R = readRecords(logFile);
N = numel(R.t);
assert(N > 20, 'too few sweeps (%d)', N);
t = R.t - R.t(1);
raw = [R.x, R.y];
ekf = [R.ex, R.ey];                       % pinned/EKF output
haveE = all(isfinite(ekf), 2);
trk = ekf; trk(~haveE, :) = raw(~haveE, :);

% Smoothed speed from the EKF track
sp = [0; vecnorm(diff(trk), 2, 2)] ./ max([1; diff(t)], 1e-3);
spS = movmedian(sp, 7);
moving = spS > opts.stillSpeed;
% morphological close/open so brief glitches don't fragment segments
moving = movmax(movmin(double(moving), 5), 5) > 0.5;

% Segment into runs
seg = segRuns(moving, t);
still = seg([seg.moving] == 0 & [seg.dur] >= opts.minStillS);
move  = seg([seg.moving] == 1 & [seg.dur] >= 0.5);

fprintf('\n=== motion_check: %s ===\n', logFile);
fprintf('%d sweeps, %.0f s, %.1f Hz;  moving %.0f%% of the time\n', ...
    N, t(end), (N-1)/t(end), 100*mean(moving));

% Still-dwell stability (raw scatter vs EKF/pinned scatter)
if ~isempty(still)
    rawSc = []; ekfSc = [];
    for s = still
        idx = s.i0:s.i1;
        rawSc(end+1) = 1000*median(vecnorm(raw(idx,:) - median(raw(idx,:),1),2,2)); %#ok<AGROW>
        e = trk(idx,:); ekfSc(end+1) = 1000*median(vecnorm(e - median(e,1),2,2)); %#ok<AGROW>
    end
    fprintf('STILL dwells: %d   scatter raw %.0f mm -> output %.0f mm  (pin/ZUPT effect)\n', ...
        numel(still), median(rawSc), median(ekfSc));
    out.stillCentres = arrayfun(@(s) median(trk(s.i0:s.i1,:),1), still, 'uni', 0);
end

% Between-dwell displacements (your known ~50 cm moves)
if numel(still) >= 2
    C = cell2mat(out.stillCentres');
    d = vecnorm(diff(C), 2, 2);
    fprintf('Between-dwell moves (m): %s\n', mat2str(round(d',2)));
    fprintf('  (compare against your known step size, e.g. 0.50 m)\n');
    out.moves = d;
end

% Motion lag: EKF track vs raw during moving segments
if ~isempty(move)
    lag = [];
    for s = move
        idx = s.i0:s.i1;
        lag(end+1) = 1000*median(vecnorm(trk(idx,:) - raw(idx,:),2,2),'omitnan'); %#ok<AGROW>
    end
    fprintf('MOVING segments: %d   EKF-vs-raw offset (lag proxy) median %.0f mm\n', ...
        numel(move), median(lag));
    % path smoothness: normalised jerk of the output track vs raw
    fprintf('Path jerk (lower=smoother): raw %.2f  output %.2f  (a.u.)\n', ...
        jerk(raw(moving,:), t(moving)), jerk(trk(moving,:), t(moving)));
end

% Pin coverage (from ex,ey freezing exactly between sweeps while still)
frozen = [false; all(diff(ekf,1,1) == 0, 2) & haveE(2:end)];
fprintf('Pinned samples: %.0f%% overall; %.0f%% of STILL samples, %.0f%% of MOVING samples\n', ...
    100*mean(frozen), 100*mean(frozen(~moving)), 100*mean(frozen(moving)));
fprintf('  (want high while still, ~0 while moving)\n');

out.t = t; out.raw = raw; out.trk = trk; out.moving = moving; out.frozen = frozen;

if opts.plot
    fig = figure('Position',[0 0 1100 520]);
    tiledlayout(1,2,'TileSpacing','compact');
    nexttile; hold on; grid on; axis equal;
    plot(raw(:,1),raw(:,2),'.','Color',[.7 .7 .7],'MarkerSize',4,'DisplayName','raw');
    plot(trk(:,1),trk(:,2),'-','Color',[.85 .2 .2],'LineWidth',1.2,'DisplayName','output');
    if exist('C','var'), plot(C(:,1),C(:,2),'ko','MarkerFaceColor','y','DisplayName','dwells'); end
    legend('Location','best'); xlabel('x (m)'); ylabel('y (m)'); title('track');
    nexttile; hold on; grid on;
    plot(t, spS, 'b-'); yline(opts.stillSpeed,'r--');
    area(t, moving*max(spS), 'FaceAlpha',0.1, 'EdgeColor','none');
    xlabel('t (s)'); ylabel('speed (m/s)'); title('speed + moving mask');
    saveas(fig, replace(logFile,'.jsonl','_motion.png'));
    fprintf('Figure: %s\n', replace(logFile,'.jsonl','_motion.png'));
end
end

%% ── helpers ──────────────────────────────────────────────────────────────
function R = readRecords(f)
lines = readlines(f);
R = struct('t',[],'x',[],'y',[],'ex',[],'ey',[]);
T=[];X=[];Y=[];EX=[];EY=[];
for i = 1:numel(lines)
    if strlength(strtrim(lines(i))) == 0, continue; end
    j = jsondecode(lines(i));
    if ~isfield(j,'t_host'), continue; end
    T(end+1,1)=j.t_host; %#ok<AGROW>
    X(end+1,1)=getf(j,'x'); Y(end+1,1)=getf(j,'y'); %#ok<AGROW>
    EX(end+1,1)=getf(j,'ex'); EY(end+1,1)=getf(j,'ey'); %#ok<AGROW>
end
R.t=T; R.x=X; R.y=Y; R.ex=EX; R.ey=EY;
end

function v = getf(j,f)
if isfield(j,f) && isnumeric(j.(f)) && ~isempty(j.(f)), v=j.(f); else, v=NaN; end
end

function seg = segRuns(mask, t)
starts = find([true; diff(mask(:)) ~= 0]);
ends = [starts(2:end) - 1; numel(mask)];
seg = struct('i0', num2cell(starts'), 'i1', num2cell(ends'), ...
    'moving', num2cell(mask(starts)'), ...
    'dur', num2cell((t(ends) - t(starts))'));
end

function j = jerk(P, t)
ok = all(isfinite(P),2);
P = P(ok,:); t = t(ok);
if size(P,1) < 6, j = NaN; return; end
P = movmedian(P,3,1);
a = diff(P,2,1);                          % 2nd difference ~ accel*dt^2
j = mean(vecnorm(diff(a,1,1),2,2),'omitnan') / max(median(diff(t)),1e-3);
end
