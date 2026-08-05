function G = alignTruth(V, tHost, opts)
%ALIGNTRUTH Interpolate the Kinect truth trajectory onto UWB timestamps.
%   G = dune.alignTruth(V, tHost, opts)
%
%   V      from dune.readVisionLog
%   tHost  [N x 1] UWB packet wall-clock times (posix s, same PC as vision)
%   opts.timeOffset  seconds added to vision time (default 0; both loggers
%                    share the PC clock, refine later with 'auto' xcorr)
%   opts.lever       [dx dy] tag antenna offset from AprilTag centre in the
%                    PLATE frame (m, default [0 0]); rotated by vision yaw
%   opts.maxGapS     mask samples where nearest vision frame is further
%                    than this (occlusion / dropout), default 0.2 s
%
%   G.xy [N x 2] truth position at each UWB time (NaN where masked)
%   G.yawDeg [N x 1], G.valid [N x 1] logical, G.coverage fraction

arguments
    V struct
    tHost (:,1) double
    opts.timeOffset (1,1) double = 0
    opts.lever (1,2) double = [0 0]
    opts.maxGapS (1,1) double = 0.2
end

tv = V.t + opts.timeOffset;
G.xy     = interp1(tv, V.xy, tHost, 'linear', NaN);
% Yaw needs angle-aware interpolation (wraps at +-180).
yawU = interp1(tv, unwrap(deg2rad(V.yawDeg)), tHost, 'linear', NaN);
G.yawDeg = rad2deg(wrapToPi(yawU));

% Mask where the nearest actual vision frame is too far away in time.
nearest = interp1(tv, tv, tHost, 'nearest', 'extrap');
gap = abs(nearest - tHost);
mask = gap > opts.maxGapS | isnan(G.xy(:,1));
G.xy(mask, :) = NaN;
G.yawDeg(mask) = NaN;

% Apply lever arm: AprilTag centre -> tag antenna, rotated into world.
if any(opts.lever ~= 0)
    c = cosd(G.yawDeg); s = sind(G.yawDeg);
    G.xy = G.xy + [c .* opts.lever(1) - s .* opts.lever(2), ...
                   s .* opts.lever(1) + c .* opts.lever(2)];
end

G.valid    = ~mask;
G.coverage = mean(G.valid);
end
