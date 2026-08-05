function X = trajectoryMetrics(estXY, truthXY, opts)
%TRAJECTORYMETRICS 2D error statistics of an estimate against truth.
%   X = dune.trajectoryMetrics(estXY, truthXY, opts)
%
%   estXY, truthXY [N x 2]; rows with NaN in either are excluded.
%   opts.bounds  [xmin xmax ymin ymax] for the per-zone map
%   opts.cellM   zone grid cell size (default 1 m)
%
%   X.n, X.rmse, X.mean, X.median, X.p95, X.max  (all metres, 2D)
%   X.err [N x 1] per-sample 2D error (NaN where excluded)
%   X.zone.rmse [ny x nx], X.zone.n, X.zone.xEdges, X.zone.yEdges

arguments
    estXY (:,2) double
    truthXY (:,2) double
    opts.bounds (1,4) double = [-0.5 6.8 -0.5 3.6]
    opts.cellM (1,1) double = 1
end

d = estXY - truthXY;
err = hypot(d(:,1), d(:,2));
ok = ~isnan(err);
e = err(ok);

X.n      = numel(e);
X.rmse   = sqrt(mean(e.^2));
X.mean   = mean(e);
X.median = median(e);
X.p95    = prctile(e, 95);
X.max    = max(e);
X.err    = err;

xe = opts.bounds(1):opts.cellM:opts.bounds(2);
ye = opts.bounds(3):opts.cellM:opts.bounds(4);
if xe(end) < opts.bounds(2), xe(end+1) = opts.bounds(2); end
if ye(end) < opts.bounds(4), ye(end+1) = opts.bounds(4); end
nz = nan(numel(ye)-1, numel(xe)-1);
rz = nan(size(nz));
gx = discretize(truthXY(:,1), xe);
gy = discretize(truthXY(:,2), ye);
for iy = 1:size(nz,1)
    for ix = 1:size(nz,2)
        m = ok & gx == ix & gy == iy;
        if any(m)
            nz(iy,ix) = nnz(m);
            rz(iy,ix) = sqrt(mean(err(m).^2));
        end
    end
end
X.zone.rmse = rz; X.zone.n = nz; X.zone.xEdges = xe; X.zone.yEdges = ye;
end
