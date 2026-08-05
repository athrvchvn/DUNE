function est = estimateCameraHeight()
    %ESTIMATECAMERAHEIGHT Camera mount height by parallax — no roof access.
    %
    % WHY: floor observations from one plane only ever constrain the RATIO
    % f/h (the auditFloorScale result), so no amount of 1 m marks separates
    % the height from the focal length. But raise a feature by a KNOWN dz at
    % the same floor spot and it slides radially away from the nadir pixel
    % by the factor h/(h - dz) — f cancels exactly, so h and the nadir pixel
    % come out with no roof access and no trust in the focal length.
    %
    % Each "hop" = the same XY spot seen at floor level and at height dz.
    % Two capture modes, freely mixable:
    %   ENTER  APRILTAG hop: tag flat on the floor at the spot, then on a
    %          box/stand of accurately known height at the SAME spot
    %          (auto-detected, corners averaged over frames).
    %   c      CLICK hop: click a feature at floor level, then the same
    %          feature raised. The UWB ANCHORS work directly: click the
    %          anchor's base at the floor, then its ANTENNA TIP; dz = the
    %          antenna height above the floor (tape-measure it — the
    %          anchors are on the floor and accessible). One snapshot
    %          serves all anchors.
    %
    % Place hops WIDE (near the image edges): sensitivity grows with the
    % distance from nadir. Expect ~±3-8 cm per AprilTag/0.5 m-box hop,
    % ~±10-15 cm per clicked anchor hop (dz ~0.26 m); 5+ hops average to a
    % few cm — plenty to decide 3.4 vs 4.1 m.
    %
    % Lens distortion is corrected automatically when a stage-A calibration
    % exists (config.camcal); without it, prefer mid-radius hops — a few %
    % of barrel bends h by roughly the same few %.
    %
    % At the end the tool can (a) write h + nadir into
    % calibration_data/camera_calibration.mat so visionSystemConfig() uses
    % them immediately, and (b) combine h with your auditFloorScale centre
    % ratio to give the implied focal length.

    config = visionSystemConfig();
    cc = config.camcal;
    h0 = config.camera.height;
    f0 = config.camera.focalLength(1);
    if ~cc.available
        fprintf(['(no distortion calibration loaded — hops near the image edge ' ...
                 'will carry a few %% distortion bias)\n']);
    end

    kinectObj = imaq.VideoDevice(config.camera.deviceName, ...
                                 config.camera.deviceID, ...
                                 config.camera.colorFormat);
    kinectObj.ReturnedColorSpace = config.camera.returnedColorSpace;
    cleanupObj = onCleanup(@() release(kinectObj)); %#ok<NASGU>

    PF = []; PB = []; DZ = [];
    clickFig = [];
    while true
        ans_ = input(sprintf(['\nHop %d — (ENTER = AprilTag floor+box, c = click ' ...
                              'floor+raised e.g. anchor base/antenna, s = solve): '], ...
                             size(PF, 1) + 1), 's');
        ans_ = strtrim(ans_);
        if strcmpi(ans_, 's'), break; end

        if strcmpi(ans_, 'c')
            % ---- click mode (one reusable snapshot) ----
            if isempty(clickFig) || ~ishghandle(clickFig)
                frame = grabFrame(kinectObj);
                clickFig = figure('Name', 'Click hops: floor point, then raised point', ...
                                  'Position', [50, 50, 1500, 850]);
                imshow(frame); hold on;
            else
                figure(clickFig);
            end
            fprintf('Click the feature at FLOOR level (zoom first if needed).\n');
            [uf, vf] = ginput(1);
            plot(uf, vf, 'g+', 'MarkerSize', 12, 'LineWidth', 2);
            fprintf('Click the SAME feature RAISED (e.g. the antenna tip).\n');
            [ub, vb] = ginput(1);
            plot(ub, vb, 'r+', 'MarkerSize', 12, 'LineWidth', 2);
            plot([uf, ub], [vf, vb], 'y-', 'LineWidth', 1);
            pf = [uf, vf]; pb = [ub, vb];
        else
            % ---- AprilTag mode ----
            input('  Tag FLAT ON THE FLOOR at the spot. ENTER to capture...', 's');
            cF = captureTagCorners(kinectObj, config);
            if isempty(cF), fprintf(2, '  Tag not detected — hop discarded.\n'); continue; end
            input('  Tag RAISED on the box/stand at the SAME spot (+/- 1 cm). ENTER...', 's');
            cB = captureTagCorners(kinectObj, config);
            if isempty(cB), fprintf(2, '  Tag not detected — hop discarded.\n'); continue; end
            pf = mean(cF, 1); pb = mean(cB, 1);
        end

        if isempty(DZ)
            dzPrompt = '  dz for this hop in metres, floor to raised point: ';
        else
            dzPrompt = sprintf(['  dz for this hop in metres, floor to raised ' ...
                                'point (ENTER = %.3f): '], DZ(end));
        end
        dzs = input(dzPrompt, 's');
        if isempty(strtrim(dzs)) && ~isempty(DZ)
            dz = DZ(end);
        else
            dz = str2double(dzs);
        end
        if ~isfinite(dz) || dz <= 0.05 || dz > 2
            fprintf(2, '  Bad dz — hop discarded.\n'); continue;
        end
        pfU = undistortPts(pf, cc);
        pbU = undistortPts(pb, cc);
        slide = norm(pbU - pfU);
        if slide < 30
            fprintf(2, ['  slide only %.1f px — spot too close to the nadir for a ' ...
                        'useful hop; move it >= 1.5 m outward. Hop DISCARDED.\n'], slide);
            continue;
        end
        PF(end + 1, :) = pfU; PB(end + 1, :) = pbU; DZ(end + 1, 1) = dz; %#ok<AGROW>
        fprintf('  slide: %.1f px at radius ~%.0f px, dz = %.3f m\n', ...
                slide, norm(pfU - config.camera.principalPoint), dz);
    end

    if size(PF, 1) < 3
        fprintf(2, 'Need >= 3 hops (got %d) — aborting.\n', size(PF, 1));
        est = [];
        return;
    end

    %% Solve h + nadir pixel (f-free)
    hBest = fminbnd(@(h) parallaxCost(h, PF, PB, DZ), max(DZ) + 0.5, 10);
    [~, nadirPx, hopRes] = parallaxCost(hBest, PF, PB, DZ);

    % Per-hop implied h with the solved nadir (consistency diagnostic)
    rF = sqrt(sum((PF - nadirPx).^2, 2));
    rB = sqrt(sum((PB - nadirPx).^2, 2));
    alpha = rB ./ rF;
    hi = DZ .* alpha ./ (alpha - 1);
    hi(alpha <= 1.001) = NaN;   % raised point must move OUTWARD from nadir

    fprintf('\n=== Parallax height estimate (%d hops) ===\n', numel(hi));
    fprintf('%-5s %-8s %-10s %-12s %-10s\n', 'hop', 'dz(m)', 'slide(px)', 'implied h(m)', 'resid(px)');
    for i = 1:numel(hi)
        fprintf('%-5d %-8.3f %-10.1f %-12.3f %-10.1f\n', ...
                i, DZ(i), norm(PB(i, :) - PF(i, :)), hi(i), hopRes(i));
    end
    fprintf('\nCAMERA HEIGHT h = %.3f m   (per-hop spread: std %.3f m)\n', ...
            hBest, std(hi(isfinite(hi))));
    if std(hi(isfinite(hi))) > 0.15
        fprintf(2, ['WARNING: large per-hop spread — usually tag re-placement error ' ...
                    '(transfer the spot with a plumb line) or near-nadir hops. ' ...
                    'Treat h as provisional; redo with cleaner hops.\n']);
    end
    fprintf('Nadir pixel: (%.1f, %.1f)  |  principal point: (%.1f, %.1f)\n', ...
            nadirPx, config.camera.principalPoint);
    tiltApprox = atand(norm(nadirPx - config.camera.principalPoint) / f0);
    fprintf('Tilt from plumb (using nominal f): ~%.2f deg\n', tiltApprox);
    fprintf('Config currently assumes h = %.3f m.\n', h0);

    %% Implied focal length, combining with the auditFloorScale ratio
    rs = input(['\nCentre-zone ratio from auditFloorScale (measured/true, e.g. 0.861) ' ...
                'to derive f — ENTER to skip: '], 's');
    fEst = NaN;
    if ~isempty(strtrim(rs))
        ratio = str2double(rs);
        if isfinite(ratio) && ratio > 0.5 && ratio < 2
            % audit implied (h/f)_true = (h_cfg/f_cfg)/ratio, with the config
            % constants that were active WHEN THE AUDIT RAN (assumed = now)
            fEst = hBest * ratio * f0 / h0;
            fprintf(['Implied f = %.1f px (nominal %.0f). If these agree, the ' ...
                     'height was the whole story.\n'], fEst, f0);
        end
    end

    est = struct('h', hBest, 'nadir_px', nadirPx, 'per_hop_h', hi, ...
                 'residual_px', hopRes, 'tilt_deg_approx', tiltApprox, ...
                 'f_implied', fEst, 'n_hops', numel(hi), 'dz', DZ);

    %% Optionally make it live via camera_calibration.mat
    ccFile = fullfile(fileparts(mfilename('fullpath')), ...
                      'calibration_data', 'camera_calibration.mat');
    ans_ = input(['Write h + nadir into camera_calibration.mat so ' ...
                  'visionSystemConfig() uses them? (ENTER = yes, s = skip): '], 's');
    if ~strcmpi(strtrim(ans_), 's')
        if exist(ccFile, 'file')
            out = load(ccFile);
        else
            out = struct('version', 1, 'norm_f', f0, 'k1', 0, 'k2', 0, ...
                         'dist_center', config.camera.principalPoint, ...
                         'H_floor2norm', [], 'f_est', NaN);
        end
        out.h = hBest;
        out.nadir_px = nadirPx;
        out.tilt_deg = tiltApprox;
        out.n_hops = numel(hi);
        out.hop_rms_m = NaN;   % pixel-space solve; stage C fills the metric one
        out.date = datestr(now, 'yyyy-mm-dd HH:MM:SS'); %#ok<TNOW1,DATST>
        saveDir = fileparts(ccFile);
        if ~isempty(saveDir) && ~exist(saveDir, 'dir'), mkdir(saveDir); end
        save(ccFile, '-struct', 'out');
        fprintf(['Saved. config.camera.height is now %.3f m everywhere.\n' ...
                 'NOTE: re-run registerWorldFrameClicks / re-derive anything ' ...
                 'that used the old height.\n'], hBest);
    end
end

%% ------------------------------------------------------------------------
function [c, n, hopRes] = parallaxCost(h, PF, PB, DZ)
    %PARALLAXCOST LS residual over hops for height h; also the nadir pixel.
    % Model: PB - n = alpha .* (PF - n), alpha = h/(h - dz), all in pixels —
    % exact for a plumb camera, f-free.
    alpha = h ./ (h - DZ);
    w = 1 - alpha;                       % per-hop scalar (negative)
    Y = PB - alpha .* PF;                % Y = w .* n + noise
    n = sum(w .* Y, 1) / sum(w.^2);
    res = Y - w .* n;
    hopRes = sqrt(sum(res.^2, 2));
    c = sum(hopRes.^2);
end

function frame = grabFrame(kinectObj)
    %GRABFRAME Snapshot in the pipeline's fliplr'd RGB convention.
    frame = step(kinectObj);
    frame = frame(:, :, [3, 2, 1]);
    frame = fliplr(frame);
end

function [corners, frame] = captureTagCorners(kinectObj, config)
    %CAPTURETAGCORNERS Average the tag's corners over a few detections
    % (same as calibrateOverheadCamera's local copy).
    fam = config.apriltag.base.family;
    wantId = config.apriltag.base.id;
    acc = zeros(4, 2); nOk = 0; frame = [];
    for attempt = 1:6
        frame = grabFrame(kinectObj);
        [ids, locs] = readAprilTag(rgb2gray(frame), fam);
        j = find(ids == wantId, 1);
        if isempty(j) && isscalar(ids), j = 1; end
        if ~isempty(j)
            acc = acc + locs(:, :, j);
            nOk = nOk + 1;
            if nOk >= 3, break; end
        end
    end
    if nOk == 0, corners = []; else, corners = acc / nOk; end
end
