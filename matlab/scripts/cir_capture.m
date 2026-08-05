function out = cir_capture(port, opts)
%CIR_CAPTURE Capture DW1000 channel impulse responses through the tag.
%   out = cir_capture("COM12")                       % 10 snapshots of A5
%   out = cir_capture("COM12", anchor=1, n=30, periodS=2)
%
%   Needs the CIR-enabled TagWrover firmware (command "CIR,<aid>"). Each
%   snapshot is one TWR exchange; the tag streams the accumulator of the
%   final RANGE_REPORT frame (1016 complex taps @ 64 MHz PRF, ~1 s each).
%
%   Purpose: watch the multipath profile evolve while the position
%   "breathes" - the first-path tap should stay put; later taps moving /
%   merging into the leading edge are the multipath mechanism.
%
%   Output: waterfall figure + .mat in matlab\results\cir_<stamp>\, and the
%   per-snapshot first-path index / distance / leading-edge stats printed.

arguments
    port string = ""
    opts.anchor (1,1) double = 5
    opts.n (1,1) double = 10
    opts.periodS (1,1) double = 2
    opts.timeout (1,1) double = 10
end

ts = dune.TagSerial(port);
cleanup = onCleanup(@() delete(ts));
ts.start();
fprintf('Connected to %s. Tag rebooting...\n', ts.port);
pause(3);
ts.drain(); ts.drainEvents();

stamp = datestr(now, 'yyyymmdd_HHMMSS'); %#ok<TNOW1,DATST>
outDir = fullfile(dune.rootDir(), 'results', ['cir_' stamp]);
mkdir(outDir);

CIR = {};
FP = []; DIST = []; TT = [];
for k = 1:opts.n
    ts.send(sprintf('CIR,%d', opts.anchor));
    [cir, fpIdx, distMm] = collectCir(ts, opts.anchor, opts.timeout);
    if isempty(cir)
        fprintf('  snapshot %d: FAILED (no/partial response)\n', k);
        continue;
    end
    CIR{end+1} = cir; %#ok<AGROW>
    FP(end+1) = fpIdx; %#ok<AGROW>
    DIST(end+1) = distMm; %#ok<AGROW>
    TT(end+1) = posixtime(datetime('now', 'TimeZone', 'UTC')); %#ok<AGROW>
    m = abs(cir);
    w = m(max(1, floor(fpIdx)) : min(numel(m), floor(fpIdx) + 16));
    fprintf('  snapshot %2d: fpIdx %.2f  dist %.3f m  fpAmp %.0f  peak/fp %.2f\n', ...
        k, fpIdx, distMm / 1000, m(max(1, round(fpIdx))), max(w) / max(m(round(fpIdx)), 1));
    pause(opts.periodS);
end
nOk = numel(CIR);
fprintf('%d/%d snapshots captured.\n', nOk, opts.n);
if nOk == 0
    out = [];
    return;
end

%% Waterfall around the first path
w0 = 24;  w1 = 104;                          % taps before/after first path
W = nan(nOk, w0 + w1 + 1);
for k = 1:nOk
    c = abs(CIR{k});
    i0 = round(FP(k));
    idx = (i0 - w0) : (i0 + w1);
    v = idx >= 1 & idx <= numel(c);
    W(k, v) = c(idx(v));
end
fig = figure('Visible', 'off', 'Position', [0 0 900 500]);
imagesc(-w0:w1, 1:nOk, 20 * log10(max(W, 1)));
xline(0, 'r-', 'first path');
xlabel('tap relative to first path (~1 ns / 30 cm each)');
ylabel('snapshot #');
title(sprintf('CIR waterfall - anchor A%d (%d snapshots, dB)', opts.anchor, nOk));
colorbar;
figFile = fullfile(outDir, sprintf('cir_waterfall_A%d.png', opts.anchor));
saveas(fig, figFile);
close(fig);

out = struct('anchor', opts.anchor, 'CIR', {CIR}, 'fpIdx', FP, ...
             'dist_mm', DIST, 'thost', TT);
save(fullfile(outDir, sprintf('cir_A%d.mat', opts.anchor)), 'out');
fprintf('Saved %s and %s\n', figFile, fullfile(outDir, sprintf('cir_A%d.mat', opts.anchor)));
end

%% ── helpers ────────────────────────────────────────────────────────────────
function [cir, fpIdx, distMm] = collectCir(ts, aid, timeout)
cir = []; fpIdx = NaN; distMm = NaN;
nTaps = NaN;
segs = containers.Map('KeyType', 'double', 'ValueType', 'any');
t0 = tic;
while toc(t0) < timeout
    for e = ts.drainEvents()
        ln = e{1}.line;
        tok = regexp(ln, sprintf('^CIRHDR,%d,(\\d+),(\\d+),(-?\\d+)', aid), 'tokens', 'once');
        if ~isempty(tok)
            v = str2double(tok);
            fpIdx = v(1) / 64;               % 10.6 fixed point -> taps
            nTaps = v(2);
            distMm = v(3);
            continue;
        end
        tok = regexp(ln, sprintf('^CIRD,%d,(\\d+),([0-9A-Fa-f]+)$', aid), 'tokens', 'once');
        if ~isempty(tok)
            segs(str2double(tok{1})) = tok{2};
            continue;
        end
        if startsWith(ln, sprintf('CIREND,%d', aid))
            cir = assemble(segs, nTaps);
            return;
        end
        if startsWith(ln, 'CIR_FAIL')
            return;
        end
    end
    ts.drain();
    pause(0.05);
end
end

function cir = assemble(segs, nTaps)
cir = [];
if ~isfinite(nTaps) || segs.Count == 0, return; end
hex = '';
for seg = 0:(ceil(nTaps / 32) - 1)
    if ~isKey(segs, seg), return; end        % missing segment -> fail
    hex = [hex, segs(seg)]; %#ok<AGROW>
end
if numel(hex) < nTaps * 8, return; end
bytes = uint8(hex2dec(reshape(hex(1:nTaps * 8), 2, [])'));
vals = double(typecast(bytes(:)', 'int16'));
cir = vals(1:2:end) + 1i * vals(2:2:end);
end
