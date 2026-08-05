function S = readSessionLog(logFile, anchorIds)
%READSESSIONLOG Read one run_localization JSONL session log into arrays.
%   S = dune.readSessionLog(logFile, anchorIds)
%
%   Each JSONL record is one tag sweep. Columns of the [N x M] matrices map
%   to anchorIds (default 1:5); absent anchors are NaN.
%
%   S.tab table with per-sweep columns:
%     tag      tag id (240 = 0xF0 TagWrover/IMU, 241 = 0xF1)
%     tms      tag-side millis()
%     thost    PC wall clock (posix s); NaN for logs before 2026-07-02
%     nUsed    anchors used by the original pipeline
%     x,y      original pipeline final output (post-EKF/EMA)
%     ex,ey    original pipeline EKF position
%     lmRmse   original pipeline LM residual RMSE
%     R        [N x M] RAW ranges (m)            (r<id> fields)
%     D        [N x M] pipeline-corrected ranges (d<id> fields)
%     RX,FP    [N x M] rx / first-path power (dBm)
%     GAP      [N x M] rx-fp gap (dB), the NLOS feature
%     W        [N x M] weights the original pipeline used
%     USED     [N x M] logical, anchor used by original pipeline
%     imuYaw, imuRoll, imuPitch  (deg, NaN when no IMU tail)
%     gyroZ    (rad/s, NaN when absent)
%
%   S.meta: file, tag list, duration, record count, hasThost.

arguments
    logFile (1,1) string
    anchorIds (1,:) double = 1:5
end

lines = readlines(logFile);
lines = lines(strlength(strtrim(lines)) > 0);
N = numel(lines);
M = numel(anchorIds);

tag   = nan(N,1); tms   = nan(N,1); thost = nan(N,1); nUsed = nan(N,1);
x     = nan(N,1); y     = nan(N,1); ex    = nan(N,1); ey    = nan(N,1);
lmRmse = nan(N,1);
R   = nan(N,M); D  = nan(N,M); RX = nan(N,M); FP = nan(N,M);
GAP = nan(N,M); W  = nan(N,M); USED = false(N,M);
imuYaw = nan(N,1); imuRoll = nan(N,1); imuPitch = nan(N,1); gyroZ = nan(N,1);

nBad = 0;
events = struct('thost', {}, 'src', {}, 'line', {});
for i = 1:N
    try
        rec = jsondecode(lines(i));
    catch
        nBad = nBad + 1;
        continue
    end
    if ~isfield(rec, 'tag_id')
        % Non-sweep record (boot/reset events etc.) — keep separately.
        ev.thost = NaN; ev.src = ''; ev.line = '';
        if isfield(rec, 't_host'), ev.thost = rec.t_host; end
        if isfield(rec, 'src'),    ev.src   = rec.src;    end
        if isfield(rec, 'line'),   ev.line  = rec.line;   end
        events(end+1) = ev; %#ok<AGROW>
        continue
    end
    tag(i)   = rec.tag_id;
    tms(i)   = rec.t_ms;
    if isfield(rec, 't_host'), thost(i) = rec.t_host; end
    if isfield(rec, 'nUsed'),  nUsed(i) = rec.nUsed;  end
    if isfield(rec, 'x'),  x(i)  = rec.x;  y(i)  = rec.y;  end
    if isfield(rec, 'ex'), ex(i) = rec.ex; ey(i) = rec.ey; end
    if isfield(rec, 'rmse'), lmRmse(i) = rec.rmse; end

    for c = 1:M
        rf = sprintf('r%d', anchorIds(c));
        df = sprintf('d%d', anchorIds(c));
        if isfield(rec, rf), R(i,c) = rec.(rf); end
        if isfield(rec, df), D(i,c) = rec.(df); end
    end

    if isfield(rec, 'anchor_diag') && ~isempty(rec.anchor_diag)
        diag = rec.anchor_diag;
        if isstruct(diag)   % struct array (uniform fields)
            diag = num2cell(diag);
        end
        for k = 1:numel(diag)
            dk = diag{k};
            c = find(anchorIds == dk.id, 1);
            if isempty(c), continue; end
            if isfield(dk,'rx'),   RX(i,c)  = dk.rx;   end
            if isfield(dk,'fp'),   FP(i,c)  = dk.fp;   end
            if isfield(dk,'gap'),  GAP(i,c) = dk.gap;  end
            if isfield(dk,'w'),    W(i,c)   = dk.w;    end
            if isfield(dk,'used'), USED(i,c) = logical(dk.used); end
        end
    end

    if isfield(rec, 'imu') && isstruct(rec.imu)
        if isfield(rec.imu,'yaw'),   imuYaw(i)   = rec.imu.yaw;   end
        if isfield(rec.imu,'roll'),  imuRoll(i)  = rec.imu.roll;  end
        if isfield(rec.imu,'pitch'), imuPitch(i) = rec.imu.pitch; end
    end
    if isfield(rec, 'imu_gyro') && numel(rec.imu_gyro) == 3
        gyroZ(i) = rec.imu_gyro(3);
    end
end

ok = ~isnan(tag);
S.tab = table(tag(ok), tms(ok), thost(ok), nUsed(ok), x(ok), y(ok), ...
    ex(ok), ey(ok), lmRmse(ok), R(ok,:), D(ok,:), RX(ok,:), FP(ok,:), ...
    GAP(ok,:), W(ok,:), USED(ok,:), imuYaw(ok), imuRoll(ok), imuPitch(ok), gyroZ(ok), ...
    'VariableNames', {'tag','tms','thost','nUsed','x','y','ex','ey','lmRmse', ...
    'R','D','RX','FP','GAP','W','USED','imuYaw','imuRoll','imuPitch','gyroZ'});

S.events         = events;
S.meta.file      = char(logFile);
S.meta.anchorIds = anchorIds;
S.meta.tags      = unique(S.tab.tag)';
S.meta.nRecords  = height(S.tab);
S.meta.nBadLines = nBad;
S.meta.hasThost  = any(~isnan(S.tab.thost));
if S.meta.hasThost
    S.meta.durationS = max(S.tab.thost) - min(S.tab.thost);
else
    S.meta.durationS = (max(S.tab.tms) - min(S.tab.tms)) / 1000;
end
end
