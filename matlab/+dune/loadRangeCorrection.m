function C = loadRangeCorrection(jsonPath)
%LOADRANGECORRECTION Load the per-anchor power-bias correction, if present.
%   C = dune.loadRangeCorrection()   % [] when no range_correction.json
%
%   Model (fitted 2026-07-20 on 19 Kinect-truth spots): the DW1000 range
%   error is linear in first-path power, corr_mm = slope*fp_dBm + intercept.
%   Apply as corrected = raw - corr_mm/1000, with fp clamped to the fitted
%   validity window (extrapolation guard). Consumed by dune.solveSweep via
%   opts.rangeCorr.

if nargin < 1 || isempty(jsonPath)
    jsonPath = fullfile(dune.rootDir(), 'config', 'range_correction.json');
end
if ~exist(jsonPath, 'file')
    C = [];
    return;
end
raw = jsondecode(fileread(jsonPath));
n = numel(raw.anchors);
C.ids   = zeros(n, 1);
C.slope = zeros(n, 1);
C.icept = zeros(n, 1);
C.fpMin = zeros(n, 1);
C.fpMax = zeros(n, 1);
for k = 1:n
    a = raw.anchors(k);
    C.ids(k)   = a.id;
    C.slope(k) = a.slope_mm_per_dB;
    C.icept(k) = a.intercept_mm;
    C.fpMin(k) = a.fp_valid_dBm(1);
    C.fpMax(k) = a.fp_valid_dBm(2);
end
C.file = jsonPath;
end
