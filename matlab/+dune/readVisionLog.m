function V = readVisionLog(csvFile)
%READVISIONLOG Read a Kinect AprilTag trajectory CSV (aprilTagOdometry_3D).
%   V = dune.readVisionLog(csvFile)
%
%   Expected columns (subset used): PosixTime_s, X_m, Y_m, Z_m, Yaw_deg.
%   Returns arrays sorted by time with duplicate timestamps removed:
%     V.t [N x 1] posix s, V.xy [N x 2] m, V.z, V.yawDeg, V.rateHz

T = readtable(csvFile);
need = {'PosixTime_s','X_m','Y_m'};
for k = 1:numel(need)
    assert(ismember(need{k}, T.Properties.VariableNames), ...
        'readVisionLog:missingColumn', ...
        '%s has no %s column - is this an aprilTagOdometry_3D export?', ...
        csvFile, need{k});
end

[t, order] = sort(T.PosixTime_s);
T = T(order, :);
keep = [true; diff(t) > 0];
T = T(keep, :); t = t(keep);

V.t  = t;
V.xy = [T.X_m, T.Y_m];
V.z  = T.Z_m;
if ismember('Yaw_deg', T.Properties.VariableNames)
    V.yawDeg = T.Yaw_deg;
else
    V.yawDeg = nan(height(T), 1);
end
V.rateHz = (numel(t) - 1) / max(t(end) - t(1), eps);
V.file   = char(csvFile);
end
