function test_step3()
% Step-3 smoke test: solveSweep numerics, sentinel reject, bias, JSONL round-trip.
nFail = 0;

A = dune.loadAnchors();
tagTrue = [1.0, 0.5];
tagZ = 0.22;
r = vecnorm(A.pos - [tagTrue, tagZ], 2, 2);

% --- 1. clean sweep through parser + solver -----------------------------
line = mkLine(240, 12345, A.ids, r, -80 * ones(5,1), -81 * ones(5,1));
s = dune.parseRtlsLine(line);
assert(~isempty(s) && numel(s.ids) == 5, 'parse failed');
[p, info] = dune.solveSweep(s, A, tagZ=tagZ);
err = norm(p - tagTrue);
nFail = nFail + check(err < 0.005, sprintf('clean solve err %.1f mm', 1000*err));
nFail = nFail + check(nnz(info.used) == 5, 'all 5 anchors used');

% --- 2. sentinel rejection ----------------------------------------------
rx = -80 * ones(5,1); fp = -81 * ones(5,1);
rBad = r; rBad(3) = 55.0;                  % garbage range on A3
rx(3) = -2147483648;                       % with the sentinel marker
line = mkLine(240, 12346, A.ids, rBad, rx, fp);
s = dune.parseRtlsLine(line);
[p, info] = dune.solveSweep(s, A, tagZ=tagZ);
err = norm(p - tagTrue);
nFail = nFail + check(info.rejected(3), 'A3 sentinel rejected');
nFail = nFail + check(isnan(info.range(3)), 'A3 range excluded');
nFail = nFail + check(err < 0.005, sprintf('sentinel solve err %.1f mm', 1000*err));

% --- 3. host-side bias --------------------------------------------------
B = struct('anchorIds', A.ids, 'anchorBias', [0.2; -0.1; 0.05; 0; 0.1], ...
           'tagIds', 240, 'tagBias', 0.15);
rB = r + B.anchorBias + B.tagBias;
line = mkLine(240, 12347, A.ids, rB, -80 * ones(5,1), -81 * ones(5,1));
s = dune.parseRtlsLine(line);
[p, ~] = dune.solveSweep(s, A, tagZ=tagZ, bias=B);
err = norm(p - tagTrue);
nFail = nFail + check(err < 0.005, sprintf('bias-corrected solve err %.1f mm', 1000*err));

% --- 4. too few anchors -> no fix ---------------------------------------
line = mkLine(240, 12348, A.ids(1:2), r(1:2), -80 * ones(2,1), -81 * ones(2,1));
s = dune.parseRtlsLine(line);
p = dune.solveSweep(s, A, tagZ=tagZ);
nFail = nFail + check(all(isnan(p)), '2-anchor sweep gives NaN fix');

% --- 5. NLOS weighting kicks in -----------------------------------------
rx = -80 * ones(5,1); fp = -81 * ones(5,1);
fp(2) = -92;                               % 12 dB gap on A2 -> weight ~0.001->floor
rN = r; rN(2) = r(2) + 0.8;                % NLOS-style range inflation
line = mkLine(240, 12349, A.ids, rN, rx, fp);
s = dune.parseRtlsLine(line);
[p, info] = dune.solveSweep(s, A, tagZ=tagZ);
err = norm(p - tagTrue);
nFail = nFail + check(abs(info.w(2) - 10^(-0.9)) < 1e-3, ...
                      sprintf('A2 down-weighted (w=%.3f)', info.w(2)));
nFail = nFail + check(err < 0.05, sprintf('NLOS solve err %.1f mm', 1000*err));

% --- 6. JSONL round-trip: sweepRecord -> readSessionLog -----------------
tmp = fullfile(tempdir, 'test_step3_log.jsonl');
fid = fopen(tmp, 'w');
imuTail = ',IMU,3,1,0,0,0,0.01,0.02,9.81,0.001,0.002,0.003';
rxS = -80 * ones(5,1); rxS(3) = -2147483648;   % sentinel on A3
lines = { ...
    [mkLine(240, 100, A.ids, r, -80*ones(5,1), -81*ones(5,1)) imuTail], ...
    mkLine(240, 350, A.ids, rBad, rxS, -81*ones(5,1))};
for k = 1:numel(lines)
    s = dune.parseRtlsLine(lines{k});
    s.thost = 1789000000 + k * 0.25;
    [p, info] = dune.solveSweep(s, A, tagZ=tagZ);
    fprintf(fid, '%s\n', jsonencode(dune.sweepRecord(s, p, info, A)));
end
fclose(fid);
S = dune.readSessionLog(tmp);
nFail = nFail + check(height(S.tab) == 2, 'round-trip: 2 records read');
nFail = nFail + check(S.meta.hasThost, 'round-trip: t_host present');
nFail = nFail + check(abs(S.tab.R(1,1) - r(1)) < 5e-4, 'round-trip: raw range r1');
nFail = nFail + check(abs(S.tab.GAP(1,1) - 1) < 0.05, 'round-trip: gap');
nFail = nFail + check(all(abs([S.tab.x(1), S.tab.y(1)] - tagTrue) < 0.005), ...
                      'round-trip: position');
nFail = nFail + check(abs(S.tab.gyroZ(1) - 0.003) < 1e-9, 'round-trip: IMU gyro z');
nFail = nFail + check(isnan(S.tab.R(2,3)), 'round-trip: sentinel range absent');
delete(tmp);

% --- 7. TagSerial construction (no port open) ---------------------------
ts = dune.TagSerial("COM99");
nFail = nFail + check(ts.port == "COM99" && ts.baud == 115200, 'TagSerial ctor');
delete(ts);

if nFail == 0
    fprintf('\nALL STEP-3 SMOKE TESTS PASSED\n');
else
    error('%d smoke test(s) FAILED', nFail);
end
end

function bad = check(cond, msg)
bad = ~cond;
if cond, fprintf('  ok    %s\n', msg);
else,    fprintf('  FAIL  %s\n', msg);
end
end

function line = mkLine(tag, tms, ids, r, rx, fp)
line = sprintf('RTLS,v3,%d,%d,%d', tms, tag, numel(ids));
for k = 1:numel(ids)
    if rx(k) == -2147483648
        line = sprintf('%s,%d,%d,%d,%.1f,%d', line, ids(k), ...
                       round(1000*r(k)), -2147483648, fp(k), 100);
    else
        line = sprintf('%s,%d,%d,%.1f,%.1f,%d', line, ids(k), ...
                       round(1000*r(k)), rx(k), fp(k), 100);
    end
end
end
