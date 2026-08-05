function out = survey_fit(captureFile)
%SURVEY_FIT Per-anchor range-bias fit from an anchor self-survey capture.
%   out = survey_fit(captureFile)
%
%   Model: measured_ij = trueDist_ij + b_i + b_j  (b in metres, one per
%   anchor). True distances come from anchors.json (tape-surveyed geometry).
%   Overdetermined LS: 10 pairs, 5 unknowns. Residuals >> ranging noise
%   mean the GEOMETRY (not the delays) is wrong -> fall back to MDS check.
%
%   Also reports each bias as DW1000 antenna-delay ticks (1 tick ~ 4.69 mm
%   on the pair range, the convention used by the existing HWCALIB flow).

arguments
    captureFile (1,1) string = fullfile(dune.rootDir(), 'results', 'survey_capture.txt')
end

TICK_M = 0.00469;   % pair-range metres per antenna-delay tick

A = dune.loadAnchors();
n = numel(A.ids);

% --- parse SURVEY,v1,<a>,<b>,<dist_mm>,<ok> lines ---------------------------
raw = readlines(captureFile);
pairs = [];   % [idA idB meas_m ok]
for L = raw'
    t = split(L, ',');
    if numel(t) >= 6 && t(1) == "SURVEY" && t(2) == "v1"
        pairs(end+1, :) = [double(t(3)), double(t(4)), double(t(5))/1000, double(t(6))]; %#ok<AGROW>
    end
end
assert(~isempty(pairs), 'No SURVEY lines found in %s', captureFile);

% --- build LS system ---------------------------------------------------------
% Skip any pair naming an anchor not in the CURRENT anchors.json (e.g. a
% capture taken while the firmware still surveyed a since-dropped anchor id) —
% otherwise ia/ib come back empty and M(k,ia)=1 silently writes nothing while
% A.pos(ia,:) still participates in trueD, corrupting that row instead of
% erroring.
keep = true(size(pairs,1), 1);
for k = 1:size(pairs,1)
    keep(k) = any(A.ids == pairs(k,1)) && any(A.ids == pairs(k,2));
end
if ~all(keep)
    dropped = pairs(~keep, 1:2);
    fprintf('survey_fit: skipping %d pair(s) for anchor(s) not in anchors.json:\n', sum(~keep));
    for k = 1:size(dropped,1), fprintf('  A%d-A%d\n', dropped(k,1), dropped(k,2)); end
end
pairs = pairs(keep, :);

np = size(pairs, 1);
M = zeros(np, n);       % incidence: 1 for each anchor in the pair
d = zeros(np, 1);       % measured - true
for k = 1:np
    ia = find(A.ids == pairs(k,1));
    ib = find(A.ids == pairs(k,2));
    M(k, ia) = 1; M(k, ib) = 1;
    trueD = norm(A.pos(ia,:) - A.pos(ib,:));
    d(k) = pairs(k,3) - trueD;
end

b = M \ d;                      % per-anchor bias (m)
resid = d - M * b;              % per-pair residual after bias removal (m)

% --- report ------------------------------------------------------------------
fprintf('\nPer-anchor bias fit (positive = board measures long):\n');
fprintf('  id    bias mm    ticks (delay += to correct)\n');
for i = 1:n
    fprintf('  A%d   %+7.1f     %+6.1f\n', A.ids(i), 1000*b(i), b(i)/TICK_M);
end
fprintf('\nPer-pair table:\n');
fprintf('  pair    true m   meas m   err mm   resid mm   ok\n');
for k = 1:np
    ia = find(A.ids == pairs(k,1)); ib = find(A.ids == pairs(k,2));
    trueD = norm(A.pos(ia,:) - A.pos(ib,:));
    fprintf('  A%d-A%d   %6.3f   %6.3f   %+7.1f   %+8.1f   %3d\n', ...
        pairs(k,1), pairs(k,2), trueD, pairs(k,3), ...
        1000*(pairs(k,3)-trueD), 1000*resid(k), pairs(k,4));
end
fprintf('\nFit residual RMS: %.1f mm  (ranging noise ~10-30 mm expected;\n', 1000*rms(resid));
fprintf('much larger => anchors.json geometry is wrong, not the delays)\n');

out.pairs = pairs; out.bias = b; out.resid = resid; out.ids = A.ids;
save(fullfile(dune.rootDir(), 'results', 'survey_fit.mat'), 'out');
end
