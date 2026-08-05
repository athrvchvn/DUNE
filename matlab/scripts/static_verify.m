function out = static_verify(captureFile, opts)
%STATIC_VERIFY Post-calibration check on a static two-tag capture.
%   Solves every sweep with anchor+tag biases applied, then reports per-tag
%   position scatter and the inter-tag baseline vs the known 0.50 m.

arguments
    captureFile (1,1) string = fullfile(dune.rootDir(), 'results', 'static_capture.txt')
    opts.tagZ (1,1) double = 0.22
    opts.trueBaselineM (1,1) double = 0.50
end

A = dune.loadAnchors();
B = dune.loadAnchorBias();
n = numel(A.ids);

ab = zeros(n,1);
for c = 1:n
    k = find(B.anchorIds == A.ids(c), 1);
    if ~isempty(k), ab(c) = B.anchorBias(k); end
end

lines = readlines(captureFile);
P = {}; T = {}; tagList = [];
for L = lines'
    s = dune.parseRtlsLine(L);
    if isempty(s), continue; end
    r = nan(n,1); g = nan(n,1);
    for m = 1:numel(s.ids)
        c = find(A.ids == s.ids(m), 1);
        if ~isempty(c), r(c) = s.dist(m); g(c) = s.rx(m) - s.fp(m); end
    end
    tb = 0;
    k = find(B.tagIds == s.tag, 1);
    if ~isempty(k) && ~isnan(B.tagBias(k)), tb = B.tagBias(k); end
    [p, ~] = dune.multilaterate(A.pos, r - ab - tb, ...
        weights=dune.gapWeights(g), tagZ=opts.tagZ);
    if any(isnan(p)), continue; end
    ti = find(tagList == s.tag, 1);
    if isempty(ti), tagList(end+1) = s.tag; ti = numel(tagList); P{ti} = []; T{ti} = []; end %#ok<AGROW>
    P{ti}(end+1,:) = p;  T{ti}(end+1,1) = s.tms; %#ok<AGROW>
end

out.tags = tagList;
fprintf('\nPost-calibration static verification\n');
for ti = 1:numel(tagList)
    mu = mean(P{ti}, 1); sd = std(P{ti}, 0, 1);
    fprintf('TAG %d: n=%d  mean [%.3f %.3f]  std [%.1f %.1f] mm  scatter p95 %.1f mm\n', ...
        tagList(ti), size(P{ti},1), mu, 1000*sd, ...
        1000*prctile(vecnorm(P{ti} - mu, 2, 2), 95));
    out.pos{ti} = P{ti}; out.mu(ti,:) = mu;
end

if numel(tagList) >= 2
    % Baseline from mean positions plus spread from per-sweep nearest pairing.
    base = norm(out.mu(1,:) - out.mu(2,:));
    d12 = [];
    for k = 1:size(P{1},1)
        [~, j] = min(abs(T{2} - T{1}(k)));   % nearest-in-time partner sweep
        d12(end+1,1) = norm(P{1}(k,:) - P{2}(j,:)); %#ok<AGROW>
    end
    fprintf('BASELINE: mean %.1f mm (true %.0f) | per-sweep median %.1f mm, std %.1f mm\n', ...
        1000*base, 1000*opts.trueBaselineM, 1000*median(d12), 1000*std(d12));
    fprintf('Baseline error vs truth: %+.1f mm\n', 1000*(base - opts.trueBaselineM));
    out.baselineM = base; out.baselineSweeps = d12;
end
save(fullfile(dune.rootDir(), 'results', 'static_verify.mat'), 'out');
end
