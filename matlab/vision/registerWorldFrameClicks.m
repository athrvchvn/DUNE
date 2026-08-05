function registration = registerWorldFrameClicks(anchorZ, opts)
    %REGISTERWORLDFRAMECLICKS Register the Kinect by clicking the anchors.
    %
    % Click-based alternative to registerWorldFrame.m: instead of placing an
    % AprilTag on surveyed floor marks, you click each UWB anchor in a Kinect
    % snapshot. The anchors' world coordinates are read from
    % matlab/config/anchors.json, and since all anchor antennas sit on ONE
    % horizontal plane (z = anchorZ above the floor), the pixel<->world
    % correspondences define a homography from which the full rigid transform
    % (R, t) camera->anchor-frame is recovered using the known intrinsics.
    %
    % Trade-off vs the AprilTag method: zero extra surveying and no tag
    % placement, but accuracy is limited by how precisely you click the
    % antenna (~1 px = ~3.3 mm on the floor at 3.43 m mount height) and by
    % uncorrected lens distortion at the image edges. Check the residual
    % report and the implied camera height/tilt; if RMSE is poor, use
    % registerWorldFrame.m instead.
    %
    % Inputs:
    %   anchorZ - height of the anchor ANTENNAS above the floor (m). All
    %             anchors must be at the same height (they are, per setup).
    %   opts    - optional struct:
    %               .anchorsFile  path to anchors.json (default:
    %                             ../config/anchors.json relative to this file)
    %               .saveFile     output .mat (default:
    %                             calibration_data/world_registration.mat)
    %               .dryRun       if true, do not save (default false)
    %
    % Output: same registration struct / .mat contents as registerWorldFrame,
    % so visionSystemConfig() / transformPoseToWorld() pick it up unchanged.
    %
    % Usage:
    %   reg = registerWorldFrameClicks(0.30);   % anchors 30 cm above floor
    %
    % Procedure: a snapshot appears; click each anchor's ANTENNA (the PCB
    % antenna area at the top of the module) in the prompted ID order. Type
    % 'r' at the confirm prompt to redo all clicks.
    %
    % A metric grid is overlaid on the click figure as an orientation aid
    % (style of MMS liveGridOverlay.m): fine lines every 0.5 m (light blue),
    % coarse lines every 1.0 m (yellow), axis-aligned with pixel spacing
    % focalLength/(mountHeight - anchorZ) and an origin crosshair at the
    % nadir pixel. It assumes the straight-down mount and does NOT depend on
    % any saved registration, so it cannot be corrupted by a stale one; its
    % labels are metres from nadir, not world coordinates. After the solve
    % the grid is redrawn by projecting WORLD coordinates through the new
    % transform — if that projected grid does not roughly coincide with the
    % nadir grid, the solve is bad (anchors.json wrong or mis-clicks) and
    % saving is blocked behind a confirm prompt.
    %
    % OPTIONAL SECOND STAGE (offered after the registration solves): click
    % the two UWB tag antennas (0xF0, 0xF1; height opts.tagZ, default =
    % anchorZ). This prints their world positions, the inter-tag baseline
    % (~0.50 m check), and the expected range to every anchor (compare with
    % the live r<id> values -> instant per-tag ranging-bias check). If the
    % AprilTag is mounted mid-baseline and visible, it is auto-detected and
    % the lever arms are printed as ready-to-paste gt_align --lever values
    % (tag body frame, Rz(yaw) convention, level-unit assumption).

    if nargin < 1 || ~isscalar(anchorZ)
        error('registerWorldFrameClicks:badInput', ...
              'Give the anchor antenna height above the floor in meters, e.g. registerWorldFrameClicks(0.30)');
    end
    if nargin < 2, opts = struct(); end
    if ~isfield(opts, 'dryRun'), opts.dryRun = false; end
    if ~isfield(opts, 'anchorsFile')
        opts.anchorsFile = fullfile(fileparts(mfilename('fullpath')), ...
                                    '..', 'config', 'anchors.json');
    end
    if ~isfield(opts, 'saveFile')
        opts.saveFile = fullfile(fileparts(mfilename('fullpath')), ...
                                 'calibration_data', 'world_registration.mat');
    end

    config = visionSystemConfig();

    %% Anchor world coordinates from anchors.json
    aj = jsondecode(fileread(opts.anchorsFile));
    ids = arrayfun(@(a) a.id, aj.anchors);
    worldPoints = [arrayfun(@(a) a.x, aj.anchors), ...
                   arrayfun(@(a) a.y, aj.anchors), ...
                   repmat(anchorZ, numel(aj.anchors), 1)];
    K = size(worldPoints, 1);
    if K < 4
        error('registerWorldFrameClicks:tooFew', ...
              'Need >= 4 anchors in %s for a homography (found %d).', ...
              opts.anchorsFile, K);
    end
    centered = worldPoints(:, 1:2) - mean(worldPoints(:, 1:2), 1);
    if rank(centered, 1e-6) < 2
        error('registerWorldFrameClicks:collinear', 'Anchor positions are collinear.');
    end

    %% Grab a snapshot (same fliplr convention as the trackers/readAprilTag use)
    kinectObj = imaq.VideoDevice(config.camera.deviceName, ...
                                 config.camera.deviceID, ...
                                 config.camera.colorFormat);
    kinectObj.ReturnedColorSpace = config.camera.returnedColorSpace;
    cleanupObj = onCleanup(@() release(kinectObj));
    rgbFrame = step(kinectObj);
    rgbFrame = rgbFrame(:, :, [3, 2, 1]);   % BGR -> RGB
    rgbFrame = fliplr(rgbFrame);            % match tracker pixel convention

    %% Intrinsics (needed for the clicks AND the grid overlay)
    fx = config.camera.focalLength(1);  fy = config.camera.focalLength(2);
    cx = config.camera.principalPoint(1); cy = config.camera.principalPoint(2);

    % Grid extent: testbed workspace, widened if anchors sit outside it
    gridXR = [min(config.workspace.xRange(1), min(worldPoints(:, 1)) - 0.5), ...
              max(config.workspace.xRange(2), max(worldPoints(:, 1)) + 0.5)];
    gridYR = [min(config.workspace.yRange(1), min(worldPoints(:, 2)) - 0.5), ...
              max(config.workspace.yRange(2), max(worldPoints(:, 2)) + 0.5)];

    %% Click each anchor
    fig = figure('Name', 'Click-based world registration', ...
                 'Position', [50, 50, 1500, 850]);
    while true
        clf(fig);
        imshow(rgbFrame);
        hold on;
        gridH = drawNadirGrid(size(rgbFrame, 2), size(rgbFrame, 1), cx, cy, ...
                              fx / max(config.camera.height - anchorZ, 0.5), ...
                              anchorZ, config.camcal);
        title(sprintf(['Click each anchor ANTENNA in ID order: %s   ' ...
                       '(zoom with the magnifier first if needed, then re-select the arrow tool)'], ...
                      mat2str(ids(:)')), 'FontSize', 12);
        pixels = nan(K, 2);
        for k = 1:K
            xlabel(sprintf('>>> Click anchor 0x%02X  (world %.2f, %.2f, z=%.2f m)   [%d/%d]', ...
                           ids(k), worldPoints(k, 1), worldPoints(k, 2), anchorZ, k, K), ...
                   'FontSize', 13, 'FontWeight', 'bold', 'Color', 'red');
            [u, v] = ginput(1);
            pixels(k, :) = [u, v];
            plot(u, v, 'g+', 'MarkerSize', 14, 'LineWidth', 2);
            text(u + 12, v, sprintf('0x%02X', ids(k)), 'Color', 'green', ...
                 'FontSize', 12, 'FontWeight', 'bold');
        end
        xlabel('All anchors clicked.', 'Color', 'black');
        ans_ = input('Accept clicks? (ENTER = yes, r = redo): ', 's');
        if ~strcmpi(strtrim(ans_), 'r')
            break;
        end
    end

    %% Normalized image coordinates (lens distortion removed when calibrated)
    pixelsU = undistortPts(pixels, config.camcal);
    xn = (pixelsU(:, 1) - cx) / fx;
    yn = (pixelsU(:, 2) - cy) / fy;

    %% Homography (DLT, Hartley-normalized): [X Y 1] -> normalized [xn yn 1]
    H = dltHomography(worldPoints(:, 1:2), [xn, yn]);

    %% Decompose H = [r1, r2, r3*Za + t_wc] (world->camera, plane z = Za)
    lambda = (norm(H(:, 1)) + norm(H(:, 2))) / 2;
    % Points must be in front of the camera (positive depth):
    testDepth = H(:, 3)' * [0; 0; 1];   % ~ z-component of a plane point in cam frame
    if testDepth < 0
        lambda = -lambda;
    end
    r1 = H(:, 1) / lambda;
    r2 = H(:, 2) / lambda;
    r3 = cross(r1, r2);
    % Nearest proper rotation (orthonormalize)
    [U, ~, V] = svd([r1, r2, r3]);
    R_wc = U * diag([1, 1, sign(det(U * V'))]) * V';
    t_wc = H(:, 3) / lambda - R_wc(:, 3) * anchorZ;

    % Camera -> world (the convention world_registration.mat stores)
    R = R_wc';
    t = -R_wc' * t_wc;

    %% Residuals: map click rays back onto the anchor plane, compare world XY
    camPoints = (R_wc * worldPoints' + t_wc)';   % anchor positions in cam frame
    mappedXY = nan(K, 2);
    for k = 1:K
        ray = [xn(k); yn(k); 1];
        % Intersect camera ray with world plane z = anchorZ:
        % world(s) = R * (s * ray) + t ; solve world_z = anchorZ for s
        dirW = R * ray;
        s = (anchorZ - t(3)) / dirW(3);
        pw = R * (s * ray) + t;
        mappedXY(k, :) = pw(1:2)';
    end
    residuals = [mappedXY - worldPoints(:, 1:2), zeros(K, 1)];
    perPoint = sqrt(sum(residuals(:, 1:2).^2, 2));
    rmse = sqrt(mean(perPoint.^2));

    %% Report
    fprintf('\n=== Click-based registration result ===\n');
    fprintf('R_cam2world =\n'); disp(R);
    fprintf('t_cam2world = [%.4f, %.4f, %.4f] m\n', t);
    tiltDeg = acosd(max(-1, min(1, abs(R(3, 3)))));
    fprintf('Implied camera height: %.3f m (config says %.3f m) | tilt from plumb: %.2f deg\n', ...
            t(3), config.camera.height, tiltDeg);
    if abs(t(3) - config.camera.height) > 0.25
        warning(['registerWorldFrameClicks:height - implied camera height differs ' ...
                 'from the measured mount height by %.2f m. A click or an ' ...
                 'anchors.json coordinate is probably wrong.'], ...
                abs(t(3) - config.camera.height));
    end
    fprintf('%-8s %-22s %-22s %-10s\n', 'Anchor', 'World XY (m)', 'Mapped XY (m)', 'Err (mm)');
    for k = 1:K
        fprintf('0x%02X     (%6.3f, %6.3f)      (%6.3f, %6.3f)      %7.1f\n', ...
                ids(k), worldPoints(k, 1:2), mappedXY(k, :), perPoint(k) * 1000);
    end
    fprintf('RMSE: %.1f mm | max: %.1f mm\n', rmse * 1000, max(perPoint) * 1000);
    fprintf(['NOTE: residuals here are self-consistency of the fit, seen through ' ...
             'your clicks;\nfor an independent check, place the AprilTag at one ' ...
             'known spot and compare.\n']);
    if rmse > 0.03
        warning(['registerWorldFrameClicks:highResidual - RMSE %.1f mm is high. ' ...
                 'Re-click more carefully (zoom in), verify anchors.json, or use ' ...
                 'the AprilTag method (registerWorldFrame.m).'], rmse * 1000);
    end

    %% Redraw the grid by projecting WORLD coordinates through the new
    %  transform. For this straight-down mount it should roughly coincide
    %  with the nadir grid shown during clicking; a rotated / shrunken /
    %  sheared grid means the solve is bad (anchors.json vs clicks mismatch).
    figure(fig);
    delete(gridH(ishghandle(gridH)));
    gridH = drawWorldGrid(R, t, fx, fy, cx, cy, anchorZ, gridXR, gridYR, ...
                          'SOLVED registration — verify vs room', config.camcal);
    fprintf(['Grid redrawn from the solved transform (world axes). It should nearly\n' ...
             'match the nadir grid shown while clicking; if not, do NOT save.\n']);
    regSuspect = rmse > 0.05 || abs(t(3) - config.camera.height) > 0.25;

    %% Optional: click the UWB tag antennas + auto-detect the AprilTag
    % Gives (a) a one-off ground-truth snapshot of each UWB tag antenna
    % position (compare expected vs live ranges -> per-tag bias check),
    % (b) the inter-tag baseline check (~0.50 m), and (c) if the AprilTag is
    % mounted mid-baseline and visible, the lever arms in the exact format
    % python/analysis/gt_align.py expects for --lever (tag body frame,
    % rotated by Rz(yaw), level-unit assumption).
    uwbTags = struct('ids', [], 'worldXY', [], 'pixels', [], 'tagZ', NaN);
    lever = struct('available', false);
    if ~isfield(opts, 'uwbTagIds'), opts.uwbTagIds = [240, 241]; end   % 0xF0, 0xF1
    if ~isfield(opts, 'tagZ'), opts.tagZ = anchorZ; end
    ans2 = input('Also click the UWB tag antennas (unit in view)? (ENTER = yes, s = skip): ', 's');
    if ~strcmpi(strtrim(ans2), 's')
        nT = numel(opts.uwbTagIds);
        tagPix = nan(nT, 2);
        tagXY  = nan(nT, 2);
        figure(fig); hold on;
        if abs(opts.tagZ - anchorZ) > 1e-9
            % Tag clicks intersect the z = tagZ plane; move the grid there
            delete(gridH(ishghandle(gridH)));
            gridH = drawWorldGrid(R, t, fx, fy, cx, cy, opts.tagZ, ...
                                  gridXR, gridYR, 'SOLVED registration', ...
                                  config.camcal); %#ok<NASGU>
        end
        for k = 1:nT
            xlabel(sprintf('>>> Click UWB tag 0x%02X antenna (z = %.2f m)   [%d/%d]', ...
                           opts.uwbTagIds(k), opts.tagZ, k, nT), ...
                   'FontSize', 13, 'FontWeight', 'bold', 'Color', 'blue');
            [u, v] = ginput(1);
            tagPix(k, :) = [u, v];
            plot(u, v, 'bx', 'MarkerSize', 14, 'LineWidth', 2);
            text(u + 12, v, sprintf('0x%02X', opts.uwbTagIds(k)), 'Color', 'cyan', ...
                 'FontSize', 12, 'FontWeight', 'bold');
            uvU = undistortPts([u, v], config.camcal);
            xnT = (uvU(1) - cx) / fx;
            ynT = (uvU(2) - cy) / fy;
            dirW = R * [xnT; ynT; 1];
            sT = (opts.tagZ - t(3)) / dirW(3);
            pw = R * (sT * [xnT; ynT; 1]) + t;
            tagXY(k, :) = pw(1:2)';
        end
        xlabel('Done.', 'Color', 'black');

        fprintf('\n=== UWB tag snapshot (z = %.2f m plane) ===\n', opts.tagZ);
        for k = 1:nT
            fprintf('  tag 0x%02X : world (%.3f, %.3f) m\n', opts.uwbTagIds(k), tagXY(k, :));
        end
        if nT == 2
            fprintf('  inter-tag baseline: %.3f m (expect ~0.500)\n', ...
                    norm(tagXY(1, :) - tagXY(2, :)));
        end
        fprintf('  Expected anchor ranges (m) - compare with live r<id> values:\n');
        fprintf('    %-8s', 'tag');
        fprintf('  0x%02X ', ids);
        fprintf('\n');
        for k = 1:nT
            fprintf('    0x%02X  ', opts.uwbTagIds(k));
            for a = 1:K
                fprintf('%6.3f ', norm([tagXY(k, :), opts.tagZ] - ...
                                       [worldPoints(a, 1:2), anchorZ]));
            end
            fprintf('\n');
        end
        uwbTags = struct('ids', opts.uwbTagIds, 'worldXY', tagXY, ...
                         'pixels', tagPix, 'tagZ', opts.tagZ);

        % Auto-detect the AprilTag (mounted mid-baseline) for the lever arms
        camT = nan(0, 3);
        camRs = {};
        grayFrame = rgb2gray(rgbFrame);   % rgbFrame is already flipped
        for f = 1:15
            if f > 1
                frm = step(kinectObj);
                frm = frm(:, :, [3, 2, 1]);
                grayFrame = rgb2gray(fliplr(frm));
            end
            [dIds, ~, dPoses] = readAprilTag(grayFrame, config.apriltag.base.family, ...
                                             config.camera.intrinsics, config.apriltag.base.size);
            j = find(dIds == config.apriltag.base.id, 1);
            if ~isempty(j)
                camT(end + 1, :) = dPoses(j).Translation; %#ok<AGROW>
                camRs{end + 1} = dPoses(j).R;             %#ok<AGROW>
            end
        end
        if isempty(camT)
            fprintf(['  AprilTag id %d NOT detected in view - lever arms not computed.\n' ...
                     '  Mount it and re-run, or measure the lever arms by hand.\n'], ...
                    config.apriltag.base.id);
        else
            camPosTag = median(camT, 1);
            camRotTag = camRs{ceil(numel(camRs) / 2)};
            R_flip = [1, 0, 0; 0, -1, 0; 0, 0, -1];
            cWorld = (R * camPosTag(:) + t)';
            wRot = R * camRotTag * R_flip';
            eul = rotm2eul(wRot, 'XYZ');
            yaw = eul(3);
            fprintf('  AprilTag centre: world (%.3f, %.3f, %.3f) m, yaw %.1f deg (%d frames)\n', ...
                    cWorld, rad2deg(yaw), size(camT, 1));
            % lever_body = Rz(-yaw) * (antenna_world - tag_centre_world)
            cyw = cos(yaw);
            syw = sin(yaw);
            levers = nan(nT, 3);
            for k = 1:nT
                dxy = tagXY(k, :) - cWorld(1:2);
                levers(k, :) = [ cyw * dxy(1) + syw * dxy(2), ...
                                -syw * dxy(1) + cyw * dxy(2), ...
                                 opts.tagZ - cWorld(3)];
                fprintf('  gt_align: --tag-id %d --lever %.3f,%.3f,%.3f\n', ...
                        opts.uwbTagIds(k), levers(k, :));
            end
            lever = struct('available', true, 'apriltagWorld', cWorld, ...
                           'yaw_rad', yaw, 'uwbTagIds', opts.uwbTagIds, ...
                           'levers_body', levers);
        end
    end

    %% Sanity gate: never silently overwrite the calibration with a bad solve
    if ~opts.dryRun && regSuspect
        fprintf(2, ['\nSANITY CHECKS FAILED: RMSE %.0f mm (limit 50), implied camera ' ...
                    'height %.2f m vs mounted %.2f m (limit +/-0.25).\n' ...
                    'Saving this would corrupt clickTagTruth / transformPoseToWorld.\n' ...
                    'Likely cause: anchors.json does not match the clicked anchors ' ...
                    '(stale coordinates or wrong click order).\n'], ...
                rmse * 1000, t(3), config.camera.height);
        ansSave = input('Save anyway? (y = yes, ENTER = no): ', 's');
        if ~strcmpi(strtrim(ansSave), 'y')
            opts.dryRun = true;
            fprintf('NOT saved. Fix anchors.json / re-click, then re-run.\n');
        end
    end

    %% Save (same schema as registerWorldFrame.m, plus the click extras)
    registration = struct('R_cam2world', R, 't_cam2world', t, ...
                          'rmse_m', rmse, 'residuals_m', residuals, ...
                          'worldPoints', worldPoints, 'camPoints', camPoints, ...
                          'pixels', pixels, 'anchorIds', ids, ...
                          'anchorZ', anchorZ, 'method', 'clicks', ...
                          'uwbTags', uwbTags, 'lever', lever, ...
                          'registeredDate', datestr(now, 'yyyy-mm-dd HH:MM:SS')); %#ok<TNOW1,DATST>
    if ~opts.dryRun
        saveDir = fileparts(opts.saveFile);
        if ~isempty(saveDir) && ~exist(saveDir, 'dir')
            mkdir(saveDir);
        end
        R_cam2world = R; t_cam2world = t; rmse_m = rmse; residuals_m = residuals;
        registeredDate = registration.registeredDate;
        uwbTagsSnapshot = uwbTags; leverArms = lever;
        save(opts.saveFile, 'R_cam2world', 't_cam2world', 'rmse_m', ...
             'residuals_m', 'worldPoints', 'camPoints', 'registeredDate', ...
             'uwbTagsSnapshot', 'leverArms');
        fprintf('Saved to %s\nvisionSystemConfig() will now use this transform automatically.\n', ...
                opts.saveFile);
    else
        fprintf('Dry run - nothing saved.\n');
    end
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
    H = H / norm(H(:, 1));   % fix overall scale sign-agnostically
end

function T = normTransform(pts)
    %NORMTRANSFORM Hartley normalization: centroid to origin, mean dist sqrt(2).
    mu = mean(pts, 1);
    d = mean(sqrt(sum((pts - mu).^2, 2)));
    if d < eps, d = 1; end
    s = sqrt(2) / d;
    T = [s, 0, -s * mu(1); 0, s, -s * mu(2); 0, 0, 1];
end

function h = drawNadirGrid(imgW, imgH, cx, cy, ppm, zPlane, cc)
    %DRAWNADIRGRID Metric grid for the straight-down camera, in the style of
    % MMS examples/liveGridOverlay.m: axis-aligned lines with pixel spacing
    % ppm = focalLength / (mountHeight - zPlane), origin crosshair at the
    % nadir pixel. Fine lines every 0.5 m (light blue), coarse every 1.0 m
    % (yellow, labelled in metres FROM NADIR — these are distances, not
    % world coordinates). Independent of any saved registration. When an
    % in-situ calibration (cc = config.camcal) is loaded, the grid is
    % centred on the calibrated nadir pixel and each line is bent through
    % distortPts() so it lands correctly on the RAW image.
    FINE = 0.5;
    COARSE = 1.0;
    colFine   = [0.25, 0.75, 0.95];
    colCoarse = [1.00, 0.80, 0.10];
    colOrigin = [1.00, 1.00, 0.00];
    finePx   = FINE * ppm;
    coarsePx = COARSE * ppm;
    h = gobjects(0);

    % Grid origin: calibrated nadir pixel when known, else principal point
    ox = cx; oy = cy;
    if isstruct(cc) && isfield(cc, 'available') && cc.available && ...
            isfield(cc, 'nadir_px') && numel(cc.nadir_px) == 2 && ...
            all(isfinite(cc.nadir_px))
        ox = cc.nadir_px(1); oy = cc.nadir_px(2);
    end
    nS = 25;   % samples per line (distortion bends them into curves)
    vline = @(x) distortPts([repmat(x, nS, 1), linspace(1, imgH, nS)'], cc);
    hline = @(y) distortPts([linspace(1, imgW, nS)', repmat(y, nS, 1)], cc);

    % Fine lines first (coarse drawn on top, like the example's minor/major)
    for x = mod(ox, finePx):finePx:imgW
        q = vline(x);
        h(end + 1) = plot(q(:, 1), q(:, 2), '-', 'Color', colFine, ...
                          'LineWidth', 0.5, 'PickableParts', 'none'); %#ok<AGROW>
    end
    for y = mod(oy, finePx):finePx:imgH
        q = hline(y);
        h(end + 1) = plot(q(:, 1), q(:, 2), '-', 'Color', colFine, ...
                          'LineWidth', 0.5, 'PickableParts', 'none'); %#ok<AGROW>
    end

    % Coarse lines with metre labels (signed distance from nadir)
    for k = -floor(ox / coarsePx):floor((imgW - ox) / coarsePx)
        x = ox + k * coarsePx;
        q = vline(x);
        h(end + 1) = plot(q(:, 1), q(:, 2), '-', 'Color', colCoarse, ...
                          'LineWidth', 1.2, 'PickableParts', 'none'); %#ok<AGROW>
        lp = distortPts([x + 4, 16], cc);
        h(end + 1) = text(lp(1), lp(2), sprintf('%+g m', k * COARSE), ...
                          'Color', colCoarse, 'FontSize', 9, 'FontWeight', 'bold', ...
                          'PickableParts', 'none', 'Clipping', 'on'); %#ok<AGROW>
    end
    for k = -floor(oy / coarsePx):floor((imgH - oy) / coarsePx)
        y = oy + k * coarsePx;
        q = hline(y);
        h(end + 1) = plot(q(:, 1), q(:, 2), '-', 'Color', colCoarse, ...
                          'LineWidth', 1.2, 'PickableParts', 'none'); %#ok<AGROW>
        lp = distortPts([4, y - 10], cc);
        h(end + 1) = text(lp(1), lp(2), sprintf('%+g m', k * COARSE), ...
                          'Color', colCoarse, 'FontSize', 9, 'FontWeight', 'bold', ...
                          'PickableParts', 'none', 'Clipping', 'on'); %#ok<AGROW>
    end

    % Nadir crosshair
    op = distortPts([ox, oy], cc);
    h(end + 1) = plot(op(1), op(2), '+', 'Color', colOrigin, 'MarkerSize', 18, ...
                      'LineWidth', 2.5, 'PickableParts', 'none');
    h(end + 1) = text(op(1) + 8, op(2) - 14, 'nadir', 'Color', colOrigin, ...
                      'FontSize', 10, 'FontWeight', 'bold', ...
                      'PickableParts', 'none', 'Clipping', 'on');

    h(end + 1) = text(15, 25, sprintf(['grid 0.5 m  (nadir approx, z = %.2f m ' ...
                      '— metres from nadir, NOT world axes)'], zPlane), ...
                      'Color', colFine, 'FontSize', 10, 'FontWeight', 'bold', ...
                      'BackgroundColor', 'k', 'Margin', 1, 'Clipping', 'on');
    h(end + 1) = text(15, 55, 'grid 1.0 m', ...
                      'Color', colCoarse, 'FontSize', 10, 'FontWeight', 'bold', ...
                      'BackgroundColor', 'k', 'Margin', 1, 'Clipping', 'on');
end

function h = drawWorldGrid(R, t, fx, fy, cx, cy, zPlane, xRange, yRange, note, cc)
    %DRAWWORLDGRID Overlay a world-frame XY grid (plane z = zPlane) on the
    % current image axes. Fine lines every 0.5 m (light blue), coarse lines
    % every 1.0 m (yellow, labelled with their world coordinate). R/t are
    % camera->world (the registration convention); samples behind the camera
    % are masked out. Projected points are bent through distortPts(cc) so
    % they land correctly on the RAW image when a calibration is loaded.
    % Returns every handle so the caller can delete/redraw.
    FINE = 0.5;
    COARSE = 1.0;
    colFine   = [0.25, 0.75, 0.95];   % light blue — 0.5 m
    colCoarse = [1.00, 0.80, 0.10];   % yellow     — 1.0 m
    R_wc = R';
    t_wc = -R' * t(:);
    nS = 60;
    h = gobjects(0);
    xTicks = (ceil(xRange(1) / FINE) : floor(xRange(2) / FINE)) * FINE;
    yTicks = (ceil(yRange(1) / FINE) : floor(yRange(2) / FINE)) * FINE;
    for x = xTicks
        [u, v] = project([repmat(x, nS, 1), ...
                          linspace(yRange(1), yRange(2), nS)', ...
                          repmat(zPlane, nS, 1)]);
        h = [h, drawOne(u, v, x, sprintf('x=%g', x))]; %#ok<AGROW>
    end
    for y = yTicks
        [u, v] = project([linspace(xRange(1), xRange(2), nS)', ...
                          repmat(y, nS, 1), ...
                          repmat(zPlane, nS, 1)]);
        h = [h, drawOne(u, v, y, sprintf('y=%g', y))]; %#ok<AGROW>
    end
    h(end + 1) = text(15, 25, sprintf('grid 0.5 m  (z = %.2f m, %s)', zPlane, note), ...
                      'Color', colFine, 'FontSize', 10, 'FontWeight', 'bold', ...
                      'BackgroundColor', 'k', 'Margin', 1, 'Clipping', 'on');
    h(end + 1) = text(15, 55, 'grid 1.0 m', ...
                      'Color', colCoarse, 'FontSize', 10, 'FontWeight', 'bold', ...
                      'BackgroundColor', 'k', 'Margin', 1, 'Clipping', 'on');

    function [u, v] = project(pw)
        % World points (Nx3) -> pixel coords in the click/registration
        % convention (the fliplr'd frame the intrinsics are applied to),
        % then re-distorted to match the raw image
        pc = R_wc * pw' + t_wc;
        pc(:, pc(3, :) < 0.1) = NaN;     % behind / grazing the camera
        u = (fx * pc(1, :) ./ pc(3, :) + cx)';
        v = (fy * pc(2, :) ./ pc(3, :) + cy)';
        q = distortPts([u, v], cc);
        u = q(:, 1);
        v = q(:, 2);
    end

    function hh = drawOne(u, v, coord, label)
        coarse = abs(coord / COARSE - round(coord / COARSE)) < 1e-9;
        if coarse
            c = colCoarse; lw = 1.1;
        else
            c = colFine;   lw = 0.5;
        end
        hh = plot(u, v, '-', 'Color', c, 'LineWidth', lw, 'PickableParts', 'none');
        if coarse
            ok = find(isfinite(u) & isfinite(v));
            if ~isempty(ok)
                % Label at the visible sample nearest the image centre
                [~, mi] = min((u(ok) - cx).^2 + (v(ok) - cy).^2);
                hh(end + 1) = text(u(ok(mi)) + 4, v(ok(mi)) - 10, label, ...
                                   'Color', c, 'FontSize', 8, ...
                                   'PickableParts', 'none', 'Clipping', 'on');
            end
        end
    end
end
