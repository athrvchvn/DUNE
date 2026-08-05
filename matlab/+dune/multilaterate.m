function [pos, info] = multilaterate(anchorPos, ranges, opts)
%MULTILATERATE Weighted Levenberg-Marquardt range multilateration (2D).
%   [pos, info] = dune.multilaterate(anchorPos, ranges, opts)
%
%   anchorPos [M x 3]  anchor antenna positions (m)
%   ranges    [M x 1]  measured ranges (m); NaN = anchor absent this sweep
%   opts.weights [M x 1] per-anchor weights (default all 1)
%   opts.tagZ    fixed tag antenna height (m, default 0.22 = anchor plane)
%   opts.x0      [1 x 2] initial guess (default weighted anchor centroid)
%   opts.gateK   MAD outlier gate factor (default 3; 0 disables the re-solve)
%   opts.maxIter LM iterations (default 50)
%
%   pos  [1 x 2]  solved x,y (NaN if <3 usable ranges or diverged)
%   info.resid   [M x 1] signed residuals pred-meas (NaN where unused)
%   info.used    [M x 1] logical, anchors in the final solve
%   info.rmse    weighted residual RMSE of the final solve
%   info.iters   LM iterations used
%   info.cov     [2 x 2] position covariance estimate

arguments
    anchorPos (:,3) double
    ranges (:,1) double
    opts.weights (:,1) double = ones(size(ranges))
    opts.tagZ (1,1) double = 0.22
    opts.x0 double = []
    opts.gateK (1,1) double = 3
    opts.maxIter (1,1) double = 50
end

M = numel(ranges);
sel = ~isnan(ranges) & opts.weights > 0;
info.resid = nan(M,1); info.used = false(M,1);
info.rmse = NaN; info.iters = 0; info.cov = nan(2);
pos = [NaN NaN];
if nnz(sel) < 3, return; end

% Degenerate-geometry gate: with (near-)collinear anchors the 2D solve has a
% mirror ambiguity and a singular normal matrix (e.g. only the three top-edge
% anchors answering). Require >=0.3 m of spread off the principal line.
sv = svd(anchorPos(sel,1:2) - mean(anchorPos(sel,1:2), 1));
if sv(2) < 0.3, return; end

% First solve on all available ranges, then optional MAD gate + re-solve.
[p, r, it, C] = lmSolve(anchorPos(sel,:), ranges(sel), opts.weights(sel), ...
                        opts.tagZ, opts.x0, opts.maxIter);
used = sel;
if opts.gateK > 0 && nnz(sel) > 3 && all(isfinite(p))
    madSig = 1.4826 * mad(r, 1);
    keep = abs(r) <= max(opts.gateK * madSig, 0.02);  % floor: never gate <2 cm residuals
    if any(~keep) && nnz(keep) >= 3
        idx = find(sel);
        used = false(M,1); used(idx(keep)) = true;
        [p, r, it, C] = lmSolve(anchorPos(used,:), ranges(used), ...
                                opts.weights(used), opts.tagZ, p, opts.maxIter);
    end
end

if all(isfinite(p))
    pos = p;
    info.resid(used) = r;
    info.used  = used;
    w = opts.weights(used);
    info.rmse  = sqrt(sum((w .* r).^2) / sum(w.^2));
    info.iters = it;
    info.cov   = C;
end
end

function [p, resid, iter, C] = lmSolve(A, d, w, tagZ, x0, maxIter)
% LM minimisation of sum_i (w_i * (||p - a_i|| - d_i))^2 over p = [x y].
dz2 = (A(:,3) - tagZ).^2;
if isempty(x0) || any(~isfinite(x0))
    x0 = sum(A(:,1:2) .* w, 1) / sum(w);
end
p = x0(:)';
lambda = 1e-3;
[resid, J] = residJac(p, A, d, dz2);
cost = sum((w .* resid).^2);
iter = 0;
for iter = 1:maxIter
    Wj = J .* w;                       % weighted Jacobian rows
    H  = Wj' * Wj;
    g  = Wj' * (w .* resid);
    step = -(H + lambda * diag(diag(H))) \ g;
    pNew = p + step';
    [rNew, JNew] = residJac(pNew, A, d, dz2);
    cNew = sum((w .* rNew).^2);
    if cNew < cost
        p = pNew; resid = rNew; J = JNew; cost = cNew;
        lambda = max(lambda / 3, 1e-9);
        if norm(step) < 1e-6, break; end
    else
        lambda = lambda * 5;
        if lambda > 1e6, break; end
    end
end
% Covariance from linearisation: sigma^2 * inv(J'WJ)
dof = max(numel(d) - 2, 1);
sigma2 = cost / dof;
Wj = J .* w;
Hf = Wj' * Wj;
if rcond(Hf) > 1e-12
    C = sigma2 * inv(Hf); %#ok<MINV>
else
    C = nan(2);
end
end

function [r, J] = residJac(p, A, d, dz2)
dx = p(1) - A(:,1); dy = p(2) - A(:,2);
pred = sqrt(dx.^2 + dy.^2 + dz2);
pred = max(pred, 1e-6);
r = pred - d;
J = [dx ./ pred, dy ./ pred];
end
