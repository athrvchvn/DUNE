function out = tag_bias_fit(captureFile, opts)
%TAG_BIAS_FIT Phase C: per-tag antenna bias from a static capture.
%   out = tag_bias_fit(captureFile)
%
%   For each tag in a raw RTLS capture (static unit), applies the anchor
%   biases from anchor_bias.json, then jointly estimates the tag position
%   AND the tag's own additive range bias:
%       median_range_i - b_anchor_i = ||p - a_i|| + b_tag
%   Solved by alternating: (1) multilaterate p with current b_tag removed,
%   (2) b_tag += weighted mean residual. Converges in a few iterations
%   because a common-mode offset maps directly onto b_tag.

arguments
    captureFile (1,1) string = fullfile(dune.rootDir(), 'results', 'static_capture.txt')
    opts.tagZ (1,1) double = 0.22
end

A = dune.loadAnchors();
B = dune.loadAnchorBias();
n = numel(A.ids);

% --- parse capture -----------------------------------------------------------
lines = readlines(captureFile);
sweeps = {};
for L = lines'
    s = dune.parseRtlsLine(L);
    if ~isempty(s), sweeps{end+1} = s; end %#ok<AGROW>
end
assert(~isempty(sweeps), 'No RTLS lines in %s', captureFile);
tags = unique(cellfun(@(s) s.tag, sweeps));

out = struct([]);
for tagId = tags(:)'
    sel = cellfun(@(s) s.tag == tagId, sweeps);
    S = sweeps(sel);
    N = numel(S);

    % Per-anchor range and gap stacks.
    R   = nan(N, n);
    GAP = nan(N, n);
    for k = 1:N
        for m = 1:numel(S{k}.ids)
            c = find(A.ids == S{k}.ids(m), 1);
            if isempty(c), continue; end
            R(k,c)   = S{k}.dist(m);
            GAP(k,c) = S{k}.rx(m) - S{k}.fp(m);
        end
    end

    medR   = median(R, 1, 'omitnan')';
    stdR   = std(R, 0, 1, 'omitnan')';
    medGap = median(GAP, 1, 'omitnan')';
    nHit   = sum(~isnan(R), 1)';

    % Remove anchor biases.
    ab = zeros(n,1);
    for c = 1:n
        idx = find(B.anchorIds == A.ids(c), 1);
        if ~isempty(idx), ab(c) = B.anchorBias(idx); end
    end
    corr = medR - ab;

    % Alternating joint fit of (x, y, b_tag).
    w = dune.gapWeights(medGap);
    bTag = 0;
    p = [NaN NaN];
    for it = 1:30
        [p, info] = dune.multilaterate(A.pos, corr - bTag, ...
            weights=w, tagZ=opts.tagZ, gateK=0, x0=p);
        if any(isnan(p)), break; end
        used = info.used;
        step = sum(w(used) .* info.resid(used)) / sum(w(used));
        bTag = bTag - step;          % resid = pred - meas; meas too long -> bTag up
        if abs(step) < 1e-5, break; end
    end
    predR = sqrt(sum((A.pos(:,1:2) - p).^2, 2) + (A.pos(:,3) - opts.tagZ).^2);
    residMm = 1000 * (corr - bTag - predR);

    fprintf('\n== TAG %d ==  (%d sweeps)\n', tagId, N);
    fprintf('  position: [%.3f  %.3f]   tag bias: %+.1f mm\n', p(1), p(2), 1000*bTag);
    fprintf('  id   n     medRaw m   std mm   gap dB   resid mm (after all corrections)\n');
    for c = 1:n
        fprintf('  A%d  %4d   %7.3f    %5.1f    %5.1f    %+7.1f\n', ...
            A.ids(c), nHit(c), medR(c), 1000*stdR(c), medGap(c), residMm(c));
    end

    out(end+1).tag = tagId; %#ok<AGROW>
    out(end).pos = p; out(end).biasM = bTag;
    out(end).residMm = residMm; out(end).medGap = medGap;
    out(end).stdMm = 1000*stdR; out(end).nSweeps = N;
end
save(fullfile(dune.rootDir(), 'results', 'tag_bias_fit.mat'), 'out');
end
