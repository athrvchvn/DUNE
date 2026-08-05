function rec = sweepRecord(s, p, info, A, posEkf)
%SWEEPRECORD One solved sweep -> JSONL-ready struct (readSessionLog schema).
%   rec = dune.sweepRecord(sweep, pos, info, A)
%   rec = dune.sweepRecord(sweep, pos, info, A, posEkf)   % adds ex/ey
%   Write with: fprintf(fid, '%s\n', jsonencode(rec));
%
%   Fields mirror the historical Python logger so dune.readSessionLog and
%   replay tooling work unchanged on live-MATLAB session logs:
%     t_host, t_ms, tag_id, r<id> (raw m), d<id> (corrected m), x, y,
%     nUsed, rmse, anchor_diag [{id, rx, fp, gap, w, used, rejected}],
%     imu_status/imu_quat/imu_gyro when the sweep carries an IMU tail.

rec = struct('t_host', s.thost, 't_ms', s.tms, 'tag_id', s.tag);
for c = 1:numel(A.ids)
    if isfinite(info.range(c))
        rec.(sprintf('r%d', A.ids(c))) = round(info.range(c), 4);
        rec.(sprintf('d%d', A.ids(c))) = round(info.rangeCorr(c), 4);
    end
end
if all(isfinite(p))
    rec.x = round(p(1), 4);
    rec.y = round(p(2), 4);
end
if nargin >= 5 && numel(posEkf) == 2 && all(isfinite(posEkf))
    rec.ex = round(posEkf(1), 4);
    rec.ey = round(posEkf(2), 4);
end
rec.nUsed = nnz(info.used);
if isfinite(info.rmse), rec.rmse = round(info.rmse, 4); end

% Phase-C tag diagnostics (RTLS v4): die temperature / battery voltage
if isfield(s, 'tempC') && isfinite(s.tempC), rec.tempC = s.tempC; end
if isfield(s, 'vbat')  && isfinite(s.vbat),  rec.vbat  = s.vbat;  end

haveDiag = find(isfinite(info.range) | info.rejected)';
if ~isempty(haveDiag)
    diag = struct('id', {}, 'rx', {}, 'fp', {}, 'gap', {}, 'w', {}, ...
                  'used', {}, 'rejected', {}, 'cfo', {}, 'tex', {});
    for c = haveDiag
        % per-anchor CFO (ppm) / realised exchange time (ms) from RTLS v4
        cfo = NaN; tex = NaN;
        k = find(s.ids == A.ids(c), 1);
        % k can exceed numel(cfoPpm): RangeHold appends held entries to
        % s.ids/dist but not to the parser's cfoPpm/tex (no real exchange).
        if ~isempty(k) && isfield(s, 'cfoPpm') && k <= numel(s.cfoPpm)
            cfo = round(s.cfoPpm(k), 3); tex = s.tex(k);
        end
        diag(end+1) = struct('id', A.ids(c), 'rx', info.rx(c), ...
            'fp', info.fp(c), 'gap', round(info.gap(c), 2), ...
            'w', round(info.w(c), 3), 'used', info.used(c), ...
            'rejected', info.rejected(c), 'cfo', cfo, 'tex', tex); %#ok<AGROW>
    end
    rec.anchor_diag = diag;
end
if ~isempty(s.imu)
    rec.imu_status = s.imu.status;
    rec.imu_quat = s.imu.quat;
    rec.imu_gyro = s.imu.gyro;
end
end
