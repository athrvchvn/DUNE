function calib = calibrateOverheadCamera(opts)
    %CALIBRATEOVERHEADCAMERA In-situ calibration of the overhead Kinect.
    %
    % No checkerboard: the camera hangs ~3.4 m over a sandy floor, so classic
    % calibration is impractical. Three field stages instead, each skippable
    % (skipped stages keep their previous values from camera_calibration.mat,
    % so stages can be redone independently):
    %
    %   A. PLUMB LINES -> lens distortion (k1, k2, + centre when >= 5 lines).
    %      Lay >= 3 (ideally 6) taut strings spanning the image: along x,
    %      along y, and diagonals, with some passing near the image edges.
    %      Click >= 6 points along each. Straight strings must image
    %      straight; any bowing is radial distortion, independent of pose.
    %
    %   B. APRILTAG SWEEP -> the global pixel <-> floor-plane homography.
    %      Place the printed tag FLAT on the floor at >= 6 (ideally 12+)
    %      spots covering the whole frame incl. edges/corners. Solved
    %      jointly with each placement's pose by alternating least squares;
    %      the tag's printed size sets the scale locally everywhere. Then
    %      one or more SCALE BARS (two clicked marks a tape-measured
    %      distance apart) anchor the global scale and cross-check the
    %      printed tag size.
    %
    %   C. BOX HOPS -> camera height h, nadir pixel, tilt, focal length.
    %      Tag flat on the floor at a spot, then on a box of accurately
    %      known height at the SAME spot (+/- 1 cm); >= 3 (ideally 6) spots,
    %      spread wide. The radial parallax shift solves h and the nadir;
    %      f = (local px/m at nadir) * h.
    %
    % opts (all optional): .boxHeight (m), .saveFile, .dryRun
    %
    % Output/side effect: calibration_data/camera_calibration.mat, loaded
    % automatically by visionSystemConfig() as config.camcal; calibrated f
    % and h then supersede the nominal constants, and all click tools /
    % grid overlays apply the distortion via undistortPts()/distortPts().
    %
    % Everything runs in the fliplr'd pixel convention the whole pipeline
    % uses; the calibration is only valid in that convention.
    %
    % AFTER CALIBRATING, RE-RUN registerWorldFrameClicks: any existing world
    % registration was solved without undistortion and is inconsistent.

    if nargin < 1, opts = struct(); end
    if ~isfield(opts, 'dryRun'), opts.dryRun = false; end
    if ~isfield(opts, 'saveFile')
        opts.saveFile = fullfile(fileparts(mfilename('fullpath')), ...
                                 'calibration_data', 'camera_calibration.mat');
    end

    config = visionSystemConfig();
    pp = config.camera.principalPoint;
    % Normalization constant for the distortion/homography params. Use the
    % NOMINAL focal (not a previously calibrated f_est) so re-runs are stable.
    if isfield(config.camera, 'focalLengthNominal')
        nf = config.camera.focalLengthNominal(1);
    else
        nf = config.camera.focalLength(1);
    end

    %% Start from the previous calibration (if any) so stages merge
    calib = struct('version', 1, 'date', '', 'norm_f', nf, ...
                   'k1', 0, 'k2', 0, 'dist_center', pp, ...
                   'n_lines', 0, 'line_rms_px', NaN, ...
                   'H_floor2norm', [], 'n_sweep', 0, 'sweep_rms_px', NaN, ...
                   'scale_bars', [], 'tag_size_used', NaN, ...
                   'h', NaN, 'nadir_px', [NaN, NaN], 'nadir_floor', [NaN, NaN], ...
                   'tilt_deg', NaN, 'f_est', NaN, 'box_height', NaN, ...
                   'n_hops', 0, 'hop_rms_m', NaN);
    if exist(opts.saveFile, 'file')
        prev = load(opts.saveFile);
        fn = fieldnames(prev);
        for k = 1:numel(fn), calib.(fn{k}) = prev.(fn{k}); end
        fprintf('Loaded previous calibration (%s) — skipped stages keep its values.\n', ...
                opts.saveFile);
    end

    %% Camera
    kinectObj = imaq.VideoDevice(config.camera.deviceName, ...
                                 config.camera.deviceID, ...
                                 config.camera.colorFormat);
    kinectObj.ReturnedColorSpace = config.camera.returnedColorSpace;
    cleanupObj = onCleanup(@() release(kinectObj)); %#ok<NASGU>

    %% ================= STAGE A: plumb lines -> distortion =================
    if ~strcmpi(strtrim(input(['\nSTAGE A — plumb-line distortion. Stick / straightedge ' ...
                               'ready? (ENTER = run, s = skip): '], 's')), 's')
        % One stick, moved between lines: grab a FRESH frame for each line so a
        % single straightedge can be repositioned each time. The camera is
        % fixed, so lines captured across different frames share one distortion
        % model; prior lines are overlaid (cyan) to help you cover new areas.
        nps = input('  Points to click per line (auto-advances after that many; ENTER = 8): ', 's');
        nPts = str2double(nps);
        if isempty(strtrim(nps)) || ~isfinite(nPts) || nPts < 4, nPts = 8; end
        nPts = round(nPts);
        fig = figure('Name', 'Stage A: click along the stick (fresh frame per line)', ...
                     'Position', [50, 50, 1500, 850]);
        linesPix = {};
        while true
            n = numel(linesPix);
            if n == 0
                input('  Place the stick for line 1, then ENTER to grab a frame...', 's');
            else
                ans_ = input(sprintf(['  Line %d captured. Reposition + ENTER for the next ' ...
                                      '(r = redo last line, s = done): '], n), 's');
                cmd = lower(strtrim(ans_));
                if strcmp(cmd, 's'), break; end
                if strcmp(cmd, 'r') && ~isempty(linesPix)
                    linesPix(end) = [];
                    fprintf('  Line %d dropped — re-lay the stick and re-click it.\n', n);
                end
            end
            frame = grabFrame(kinectObj);
            figure(fig); clf; imshow(frame); hold on;
            for kk = 1:numel(linesPix)
                plot(linesPix{kk}(:, 1), linesPix{kk}(:, 2), 'c.-', 'MarkerSize', 6);
                text(linesPix{kk}(1, 1) + 8, linesPix{kk}(1, 2), sprintf('L%d', kk), ...
                     'Color', 'cyan');
            end
            fprintf(['Click %d points ALONG the stick, spread end-to-end (scroll to ' ...
                     'zoom first if needed). Advances automatically after %d clicks — ' ...
                     'no Enter needed.\n'], nPts, nPts);
            [us, vs] = ginput(nPts);
            if numel(us) < 4
                fprintf('Only %d points — need >= 4 (>= 6 recommended). Line discarded.\n', numel(us));
                continue;
            end
            linesPix{end + 1} = [us, vs]; %#ok<AGROW>
            plot(us, vs, 'y.-', 'MarkerSize', 10);
            text(us(1) + 8, vs(1), sprintf('L%d', numel(linesPix)), ...
                 'Color', 'yellow', 'FontWeight', 'bold');
        end
        nL = numel(linesPix);
        if nL < 3
            fprintf(2, 'Only %d lines — need >= 3. Stage A aborted (previous values kept).\n', nL);
        else
            % 2-param fit (centre fixed at principal point), then free centre
            o = optimset('Display', 'off', 'MaxFunEvals', 5000, 'MaxIter', 5000);
            p2 = fminsearch(@(p) plumbCost([p, pp], linesPix, nf), [0, 0], o);
            if nL >= 5
                p4 = fminsearch(@(p) plumbCost(p, linesPix, nf), [p2, pp], o);
            else
                p4 = [p2, pp];
                fprintf('(< 5 lines: distortion centre fixed at the principal point)\n');
            end
            cc0 = struct('available', true, 'k1', 0, 'k2', 0, 'dist_center', pp, 'norm_f', nf);
            ccA = struct('available', true, 'k1', p4(1), 'k2', p4(2), ...
                         'dist_center', p4(3:4), 'norm_f', nf);
            fprintf('\n%-6s %-22s %-22s\n', 'Line', 'max bow RAW (px)', 'max bow CORRECTED (px)');
            sq = 0; npts = 0;
            for i = 1:nL
                dev0 = lineMaxDev(undistortPts(linesPix{i}, cc0));
                dev1 = lineMaxDev(undistortPts(linesPix{i}, ccA));
                fprintf('L%-5d %-22.2f %-22.2f\n', i, dev0, dev1);
                q = undistortPts(linesPix{i}, ccA);
                q = q - mean(q, 1);
                [~, s_, ~] = svd(q, 0);
                sq = sq + s_(2, 2)^2; npts = npts + size(q, 1);
            end
            calib.k1 = p4(1); calib.k2 = p4(2);
            calib.dist_center = p4(3:4);
            calib.norm_f = nf;
            calib.n_lines = nL;
            calib.line_rms_px = sqrt(sq / npts);
            fprintf(['k1 = %+.5f, k2 = %+.5f, centre = (%.1f, %.1f), ' ...
                     'residual %.2f px rms\n'], calib.k1, calib.k2, ...
                    calib.dist_center, calib.line_rms_px);
        end
        if exist('fig', 'var') && ishghandle(fig), close(fig); end
    end
    ccWork = struct('available', true, 'k1', calib.k1, 'k2', calib.k2, ...
                    'dist_center', calib.dist_center, 'norm_f', calib.norm_f);

    %% ============ STAGE B: AprilTag sweep -> floor homography =============
    if ~strcmpi(strtrim(input(['\nSTAGE B — AprilTag floor sweep. Tag + board ready? ' ...
                               '(ENTER = run, s = skip): '], 's')), 's')
        S = config.apriltag.base.sizeNominal;   % PRINTED size: in-plane truth
        fprintf('Using printed tag size %.4f m (scale bars will cross-check it).\n', S);
        cornersAll = {};
        figB = figure('Name', 'Stage B: tag placements', 'Position', [50, 50, 1200, 700]);
        shown = false;
        while true
            ans_ = input(sprintf(['Placement %d: tag FLAT on the floor at a new spot ' ...
                                  '(cover edges/corners too). (ENTER = capture, s = done): '], ...
                                 numel(cornersAll) + 1), 's');
            if strcmpi(strtrim(ans_), 's'), break; end
            [corners, frame] = captureTagCorners(kinectObj, config);
            if isempty(corners)
                fprintf(2, 'Tag not detected — adjust and retry.\n');
                continue;
            end
            cornersAll{end + 1} = corners; %#ok<AGROW>
            if ishghandle(figB)
                figure(figB);
                if ~shown, imshow(frame); hold on; shown = true; end
                c = mean(corners, 1);
                plot(c(1), c(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
                text(c(1) + 10, c(2), sprintf('%d', numel(cornersAll)), ...
                     'Color', 'red', 'FontWeight', 'bold');
                drawnow;
            end
            fprintf('  captured (%d placements so far)\n', numel(cornersAll));
        end
        N = numel(cornersAll);
        if N < 4
            fprintf(2, 'Only %d placements — need >= 4. Stage B aborted (previous values kept).\n', N);
        else
            % --- alternating LS: {H} <-> {per-placement tag pose}
            template = S / 2 * [-1, -1; 1, -1; 1, 1; -1, 1];
            normC = cell(1, N);
            for i = 1:N
                und = undistortPts(cornersAll{i}, ccWork);
                normC{i} = (und - pp) / nf;
            end
            h0 = config.camera.height;
            H = [1/h0, 0, 0; 0, 1/h0, 0; 0, 0, 1];   % nadir init: floor ~ norm*h
            rmsPrev = Inf;
            for it = 1:50
                Wf = zeros(4 * N, 2); Nm = zeros(4 * N, 2);
                Hinv = inv(H); %#ok<MINV>
                for i = 1:N
                    pf = homApply(Hinv, normC{i});
                    % orthogonal Procrustes, reflection ALLOWED (the fliplr'd
                    % frame mirrors the corner winding; all placements pick
                    % the same handedness so the map stays self-consistent)
                    muT = mean(template, 1); muP = mean(pf, 1);
                    M = (pf - muP)' * (template - muT);
                    [U, ~, V] = svd(M);
                    Q = U * V';
                    w = (template - muT) * Q' + muP;
                    Wf(4*i-3:4*i, :) = w;
                    Nm(4*i-3:4*i, :) = normC{i};
                end
                H = dltHomography(Wf, Nm);
                rmsPx = nf * sqrt(mean(sum((homApply(H, Wf) - Nm).^2, 2)));
                if abs(rmsPrev - rmsPx) < 1e-4, break; end
                rmsPrev = rmsPx;
            end
            fprintf('Sweep solved: %d placements, corner reprojection %.2f px rms (%d iters).\n', ...
                    N, rmsPx, it);

            % --- scale bars: anchor the global scale to the tape measure
            bars = [];
            frame = grabFrame(kinectObj);
            if ishghandle(figB), figure(figB); clf; imshow(frame); hold on;
            else, figB = figure; imshow(frame); hold on; end
            while true
                ans_ = input(['Add a scale bar (two marks, tape-measured apart)? ' ...
                              '(ENTER = yes, s = done): '], 's');
                if strcmpi(strtrim(ans_), 's'), break; end
                fprintf('Click the TWO marks.\n');
                [ub, vb] = ginput(2);
                if numel(ub) < 2, continue; end
                plot(ub, vb, 'g+-', 'MarkerSize', 12, 'LineWidth', 2);
                D = str2double(input('True distance between them (m): ', 's'));
                if ~isfinite(D) || D <= 0, fprintf(2, 'Bad value, bar discarded.\n'); continue; end
                und = undistortPts([ub, vb], ccWork);
                pf = homApply(inv(H), (und - pp) / nf); %#ok<MINV>
                d = norm(diff(pf, 1, 1));
                bars(end + 1, :) = [D, d, D / d]; %#ok<AGROW>
                fprintf('  model says %.4f m vs true %.4f m -> scale error %+.2f%%\n', ...
                        d, D, (d / D - 1) * 100);
            end
            if ~isempty(bars)
                s = mean(bars(:, 3));
                H = H * diag([1 / s, 1 / s, 1]);
                fprintf(['Applied mean scale factor %.4f. Implied true tag size: %.4f m ' ...
                         '(printed %.4f m).\n'], s, S * s, S);
            else
                fprintf(2, ['No scale bar — global scale rests ENTIRELY on the printed ' ...
                            'tag size (%.4f m). Add one when possible.\n'], S);
            end
            calib.H_floor2norm = H;
            calib.n_sweep = N;
            calib.sweep_rms_px = rmsPx;
            calib.scale_bars = bars;
            calib.tag_size_used = S;
        end
        if exist('figB', 'var') && ishghandle(figB), close(figB); end
    end

    %% ====== STAGE C: box hops -> height, nadir, tilt, focal length ========
    if isempty(calib.H_floor2norm)
        fprintf(2, '\nSTAGE C needs the stage-B homography — run stage B first. Skipping.\n');
    elseif ~strcmpi(strtrim(input(['\nSTAGE C — box hops (tag on floor, then on the box, ' ...
                                   'same spot). (ENTER = run, s = skip): '], 's')), 's')
        if isfield(opts, 'boxHeight')
            B = opts.boxHeight;
        else
            B = str2double(input(['Box height in metres, floor surface to TAG FACE ' ...
                                  '(measure it!): '], 's'));
        end
        if ~isfinite(B) || B <= 0.05
            fprintf(2, 'Bad box height — stage C aborted.\n');
        else
            H = calib.H_floor2norm;
            Hinv = inv(H); %#ok<MINV>
            PF = []; PB = [];
            while true
                ans_ = input(sprintf(['Hop %d: tag FLAT ON THE FLOOR at the spot ' ...
                                      '(spread hops wide, incl. edges). (ENTER = capture, s = done): '], ...
                                     size(PF, 1) + 1), 's');
                if strcmpi(strtrim(ans_), 's'), break; end
                cF = captureTagCorners(kinectObj, config);
                if isempty(cF), fprintf(2, 'Tag not detected — retry.\n'); continue; end
                input('  Now the tag ON THE BOX at the SAME spot (+/- 1 cm). ENTER to capture...', 's');
                cB = captureTagCorners(kinectObj, config);
                if isempty(cB), fprintf(2, 'Tag not detected — hop discarded.\n'); continue; end
                pF = homApply(Hinv, (mean(undistortPts(cF, ccWork), 1) - pp) / nf);
                pB = homApply(Hinv, (mean(undistortPts(cB, ccWork), 1) - pp) / nf);
                PF(end + 1, :) = pF; PB(end + 1, :) = pB; %#ok<AGROW>
                fprintf('  parallax slide: %.3f m on the floor\n', norm(pB - pF));
            end
            if size(PF, 1) < 3
                fprintf(2, 'Only %d hops — need >= 3. Stage C aborted (previous values kept).\n', size(PF, 1));
            else
                hBest = fminbnd(@(h) hopCost(h, PF, PB, B), max(1.0, B + 0.3), 10);
                [cBest, nadirF] = hopCost(hBest, PF, PB, B);
                rmsM = sqrt(cBest / numel(PF));
                nadirNorm = homApply(H, nadirF);
                nadirPx = pp + nf * nadirNorm;
                tiltDeg = atand(norm(nadirNorm));
                % focal: local floor scale (px/m) at nadir, times height
                e = 1e-3;
                pix = @(w) pp + nf * homApply(H, w);
                J = [pix(nadirF + [e, 0]) - pix(nadirF); ...
                     pix(nadirF + [0, e]) - pix(nadirF)]' / e;
                fEst = sqrt(abs(det(J))) * hBest;
                fprintf(['\nheight h = %.3f m (tape says %.2f) | nadir pixel (%.1f, %.1f) | ' ...
                         'tilt %.2f deg\nfocal f = %.1f px (nominal %.0f) | hop residual %.1f mm rms\n'], ...
                        hBest, config.camera.height, nadirPx, tiltDeg, fEst, nf, rmsM * 1000);
                calib.h = hBest;
                calib.nadir_px = nadirPx;
                calib.nadir_floor = nadirF;
                calib.tilt_deg = tiltDeg;
                calib.f_est = fEst;
                calib.box_height = B;
                calib.n_hops = size(PF, 1);
                calib.hop_rms_m = rmsM;
            end
        end
    end

    %% Save
    calib.date = datestr(now, 'yyyy-mm-dd HH:MM:SS'); %#ok<TNOW1,DATST>
    if ~opts.dryRun
        saveDir = fileparts(opts.saveFile);
        if ~isempty(saveDir) && ~exist(saveDir, 'dir'), mkdir(saveDir); end
        save(opts.saveFile, '-struct', 'calib');
        fprintf(['\nSaved %s\nvisionSystemConfig() now loads it automatically ' ...
                 '(config.camcal).\nNEXT: re-run registerWorldFrameClicks — the old ' ...
                 'world registration is inconsistent with the new undistortion.\n'], ...
                opts.saveFile);
    else
        fprintf('\nDry run — nothing saved.\n');
    end
end

%% ------------------------------------------------------------------------
function frame = grabFrame(kinectObj)
    %GRABFRAME One snapshot in the pipeline's fliplr'd RGB convention.
    frame = step(kinectObj);
    frame = frame(:, :, [3, 2, 1]);   % BGR -> RGB
    frame = fliplr(frame);
end

function [corners, frame] = captureTagCorners(kinectObj, config)
    %CAPTURETAGCORNERS Average the tag's 4 corners over a few detections.
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
    if nOk == 0
        corners = [];
    else
        corners = acc / nOk;
    end
end

function c = plumbCost(p, linesPix, nf)
    %PLUMBCOST Sum of squared perpendicular residuals of undistorted lines.
    cc = struct('available', true, 'k1', p(1), 'k2', p(2), ...
                'dist_center', p(3:4), 'norm_f', nf);
    c = 0;
    for i = 1:numel(linesPix)
        q = undistortPts(linesPix{i}, cc);
        q = q - mean(q, 1);
        [~, s_, ~] = svd(q, 0);
        c = c + s_(2, 2)^2;   % Frobenius mass off the TLS line
    end
end

function d = lineMaxDev(pts)
    %LINEMAXDEV Max perpendicular deviation of points from their TLS line (px).
    q = pts - mean(pts, 1);
    [~, ~, V] = svd(q, 0);
    d = max(abs(q * V(:, 2)));
end

function q = homApply(H, pts)
    %HOMAPPLY Apply a 3x3 homography to Nx2 points.
    n = size(pts, 1);
    w = (H * [pts, ones(n, 1)]')';
    q = w(:, 1:2) ./ w(:, 3);
end

function [c, n] = hopCost(h, PF, PB, B)
    %HOPCOST Box-hop residual for camera height h; also returns the nadir.
    % Model: PB = n + alpha*(PF - n), alpha = h/(h - B).
    alpha = h / (h - B);
    n = mean((PB - alpha * PF) / (1 - alpha), 1);
    res = PB - (n + alpha * (PF - n));
    c = sum(res(:).^2);
end

function H = dltHomography(XY, xy)
    %DLTHOMOGRAPHY Homography [X Y 1] -> [x y 1], Hartley-normalized DLT.
    n = size(XY, 1);
    Tw = normTransform(XY);
    Ti = normTransform(xy);
    XYh = (Tw * [XY, ones(n, 1)]')';
    xyh = (Ti * [xy, ones(n, 1)]')';
    A = zeros(2 * n, 9);
    for k = 1:n
        X = XYh(k, :);
        x = xyh(k, 1); y = xyh(k, 2);
        A(2*k-1, :) = [-X, zeros(1, 3), x * X];
        A(2*k,   :) = [zeros(1, 3), -X, y * X];
    end
    [~, ~, V] = svd(A, 0);
    Hn = reshape(V(:, end), 3, 3)';
    H = Ti \ Hn * Tw;
    H = H / norm(H(:, 1));
end

function T = normTransform(pts)
    %NORMTRANSFORM Hartley normalization: centroid to origin, mean dist sqrt(2).
    mu = mean(pts, 1);
    d = mean(sqrt(sum((pts - mu).^2, 2)));
    if d < eps, d = 1; end
    s = sqrt(2) / d;
    T = [s, 0, -s * mu(1); 0, s, -s * mu(2); 0, 0, 1];
end
