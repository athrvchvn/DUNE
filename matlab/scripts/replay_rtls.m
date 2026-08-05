function out = replay_rtls(logFile, opts)
%REPLAY_RTLS Replay a logged session through the FULL step-3 live chain.
%   out = replay_rtls(logFile)
%   out = replay_rtls(logFile, anchors="...\anchors_rect6p3x3p0_2026-07-01.json")
%
%   Unlike replay_baseline (which re-solves the logged range matrices), this
%   reconstructs the raw "RTLS,v3,..." serial line of every sweep and pushes
%   it through dune.parseRtlsLine -> dune.solveSweep — the exact code path
%   the live COM pipeline runs — so the whole chain (parser, sentinel
%   rejection, NLOS weights, weighted LM) is exercised offline.
%
%   Old logs were recorded in the rect 6.3x3.0 layout: pass
%   anchors="matlab\config\anchors_rect6p3x3p0_2026-07-01.json" for those.
%
%   opts.anchors  anchors.json path ("" = current matlab/config/anchors.json)
%   opts.bias     ALSO subtract host-side anchor_bias.json (default false)
%   opts.tagZ     tag antenna height (m, default 0.22)
%   opts.saveDir  output dir ("" = matlab\results\replay_<session>)
%
%   out.tag/P/rmse/nUsed per sweep; prints per-tag stats and saves a
%   trajectory figure.

arguments
    logFile (1,1) string
    opts.anchors string = ""
    opts.bias (1,1) logical = false
    opts.tagZ (1,1) double = 0.22
    opts.saveDir string = ""
end

if opts.anchors ~= ""
    A = dune.loadAnchors(char(opts.anchors));
else
    A = dune.loadAnchors();
end
B = [];
if opts.bias, B = dune.loadAnchorBias(); end

S = dune.readSessionLog(logFile);
[~, sessName] = fileparts(logFile);
if opts.saveDir == ""
    opts.saveDir = fullfile(dune.rootDir(), 'results', ['replay_' char(sessName)]);
end
if ~exist(opts.saveDir, 'dir'), mkdir(opts.saveDir); end

fprintf('Replay (full live chain) — %s\n', sessName);
fprintf('Anchors: %s (%s)\n', A.file, A.layout);
fprintf('Host-side bias: %s   Records: %d\n', string(opts.bias), height(S.tab));

N = height(S.tab);
out.tag  = S.tab.tag;
out.P    = nan(N, 2);
out.rmse = nan(N, 1);
out.nUsed = zeros(N, 1);
out.nParseFail = 0;

prev = containers.Map('KeyType', 'double', 'ValueType', 'any');
for i = 1:N
    line = rtlsLineFromRecord(S.tab(i, :), S.meta.anchorIds);
    s = dune.parseRtlsLine(line);
    if isempty(s)
        out.nParseFail = out.nParseFail + 1;
        continue;
    end
    x0 = [];
    if isKey(prev, s.tag), x0 = prev(s.tag); end
    [p, info] = dune.solveSweep(s, A, bias=B, tagZ=opts.tagZ, x0=x0);
    out.P(i, :) = p;
    out.rmse(i) = info.rmse;
    out.nUsed(i) = nnz(info.used);
    if all(isfinite(p)), prev(s.tag) = p; end
end
fprintf('Parse failures: %d / %d\n', out.nParseFail, N);

%% Per-tag stats + figure
fig = figure('Visible', 'off', 'Position', [0 0 950 550]);
hold on;
cols = lines(numel(S.meta.tags));
for kt = 1:numel(S.meta.tags)
    tagId = S.meta.tags(kt);
    m = out.tag == tagId;
    P = out.P(m, :);
    solved = ~isnan(P(:, 1));
    fprintf('\nTAG %d (0x%02X): %d sweeps\n', tagId, tagId, nnz(m));
    fprintf('  solved:  %.1f%%   median LM residual RMSE %.0f mm (p95 %.0f mm)\n', ...
        100 * mean(solved), 1000 * median(out.rmse(m), 'omitnan'), ...
        1000 * prctile(out.rmse(m), 95));
    if S.meta.hasThost, tt = S.tab.thost(m); else, tt = S.tab.tms(m) / 1000; end
    dt = diff(tt); dt = dt(dt > 0 & dt < 5);
    if ~isempty(dt)
        fprintf('  update rate: median %.2f Hz\n', 1 / median(dt));
    end
    % vs the original pipeline output logged at record time
    T = S.tab(m, :);
    both = solved & ~isnan(T.x);
    if any(both)
        d = hypot(P(both, 1) - T.x(both), P(both, 2) - T.y(both));
        fprintf('  vs original pipeline x,y: median %.0f mm, p95 %.0f mm (n=%d)\n', ...
            1000 * median(d), 1000 * prctile(d, 95), nnz(both));
    end
    plot(P(:, 1), P(:, 2), '.', 'MarkerSize', 4, 'Color', cols(kt, :), ...
         'DisplayName', sprintf('tag %d (replay)', tagId));
end
plot(A.pos(:, 1), A.pos(:, 2), 'k^', 'MarkerFaceColor', 'y', 'DisplayName', 'anchors');
text(A.pos(:, 1) + 0.1, A.pos(:, 2), compose('A%d', A.ids));
axis equal; grid on; legend('Location', 'bestoutside');
title(sprintf('replay\\_rtls — %s', sessName), 'Interpreter', 'tex');
figFile = fullfile(opts.saveDir, 'trajectory.png');
saveas(fig, figFile);
close(fig);
fprintf('\nTrajectory figure: %s\n', figFile);
out.figFile = figFile;
end

function line = rtlsLineFromRecord(row, anchorIds)
% Rebuild the tag's serial line from one JSONL record (raw ranges + rx/fp).
% Missing rx/fp (pre-July-2 logs) are emitted as NaN -> gap weight 1.
have = find(~isnan(row.R));
parts = sprintf('RTLS,v3,%.0f,%d,%d', row.tms, row.tag, numel(have));
for c = have
    parts = sprintf('%s,%d,%d,%.1f,%.1f,%d', parts, anchorIds(c), ...
                    round(1000 * row.R(c)), row.RX(c), row.FP(c), 100);
end
line = parts;
end
