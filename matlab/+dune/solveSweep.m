function [pos, info] = solveSweep(sweep, A, opts)
%SOLVESWEEP One ranging sweep -> tag x,y. The single solve path, live & replay.
%   [pos, info] = dune.solveSweep(sweep, A)
%   [pos, info] = dune.solveSweep(sweep, A, x0=prevPos, bias=B)
%
%   The sweep is protocol-agnostic: any source (COM round-robin today,
%   broadcast-POLL later) that produces (anchor id, range, rx, fp) tuples at
%   one instant can feed it. dune.parseRtlsLine output is accepted directly.
%
%   sweep  struct: .tag, .ids [1xn], .dist [1xn] (m), .rx [1xn], .fp [1xn]
%   A      struct from dune.loadAnchors (ids [Mx1], pos [Mx3])
%   opts.bias    struct from dune.loadAnchorBias, or [] (default) = no
%                host-side correction. Default is OFF: the boards' NVS
%                antenna delays are the source of truth (re-tuned in step 4);
%                subtracting anchor_bias.json on top would double-correct.
%   opts.rangeCorr  struct from dune.loadRangeCorrection, or [] (default).
%                Per-anchor DW1000 power bias: corr_mm = slope*fp + icept,
%                fp clamped to the fitted window; corrected = raw - corr.
%   opts.tagZ    tag antenna height (m, default 0.22)
%   opts.x0      warm-start position [1x2] (default: weighted centroid)
%   opts.useGapWeights  NLOS soft weights from rx-fp gap (default true)
%   opts.gapThreshDb, opts.gapFloorW  -> dune.gapWeights (defaults 3, 0.05)
%   opts.gateK   MAD outlier gate -> dune.multilaterate (default 3)
%
%   pos  [1x2] world x,y (NaN NaN if <3 usable anchors / degenerate / diverged)
%   info dune.multilaterate info (resid, used, rmse, iters, cov) plus
%        per-anchor rows aligned to A.ids:
%     .range     raw range (m), NaN if absent or rejected
%     .rangeCorr bias-corrected range actually solved on
%     .rx, .fp   diagnostics (dBm)
%     .gap       rx-fp (dB)
%     .w         weight used
%     .rejected  logical, measurement dropped (rx/fp sentinel or invalid range)

arguments
    sweep (1,1) struct
    A (1,1) struct
    opts.bias = []
    opts.rangeCorr = []
    opts.tagZ (1,1) double = 0.22
    opts.x0 double = []
    opts.useGapWeights (1,1) logical = true
    opts.gapThreshDb (1,1) double = 3
    opts.gapFloorW (1,1) double = 0.05
    opts.gateK (1,1) double = 3
end

SENTINEL = -2147483648;   % DW1000 diagnostic-read error (seen on A3)

M = numel(A.ids);
range = nan(M,1); rx = nan(M,1); fp = nan(M,1);
rejected = false(M,1);
wScale = ones(M,1);           % dune.RangeHold weight for held entries

for k = 1:numel(sweep.ids)
    c = find(A.ids == sweep.ids(k), 1);
    if isempty(c), continue; end          % anchor not in the layout
    if isfield(sweep, 'wScale') && k <= numel(sweep.wScale)
        wScale(c) = sweep.wScale(k);
    end
    rx(c) = sweep.rx(k);
    fp(c) = sweep.fp(k);
    bad = sweep.rx(k) == SENTINEL || sweep.fp(k) == SENTINEL || ...
          ~isfinite(sweep.dist(k)) || sweep.dist(k) <= 0;
    if bad
        rejected(c) = true;   % range stays NaN -> excluded from the solve
        continue;
    end
    range(c) = sweep.dist(k);
end

corr = range;
if ~isempty(opts.rangeCorr)
    C = opts.rangeCorr;
    for c = 1:M
        j = find(C.ids == A.ids(c), 1);
        if isempty(j) || ~isfinite(fp(c)) || ~isfinite(corr(c)), continue; end
        f = min(max(fp(c), C.fpMin(j)), C.fpMax(j));
        corr(c) = corr(c) - (C.slope(j) * f + C.icept(j)) / 1000;
    end
end
if ~isempty(opts.bias)
    B = opts.bias;
    for c = 1:M
        j = find(B.anchorIds == A.ids(c), 1);
        if ~isempty(j), corr(c) = corr(c) - B.anchorBias(j); end
    end
    j = find(B.tagIds == sweep.tag, 1);
    if ~isempty(j) && isfinite(B.tagBias(j))
        corr = corr - B.tagBias(j);
    end
end

gap = rx - fp;
if opts.useGapWeights
    w = dune.gapWeights(gap, opts.gapThreshDb, opts.gapFloorW);
else
    w = ones(M,1);
end
w = w .* wScale;

[pos, info] = dune.multilaterate(A.pos, corr, weights=w, tagZ=opts.tagZ, ...
                                 x0=opts.x0, gateK=opts.gateK);
info.range = range;
info.rangeCorr = corr;
info.rx = rx;
info.fp = fp;
info.gap = gap;
info.w = w;
info.rejected = rejected;
end
