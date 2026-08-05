function registration = registerWorldFrame(worldPoints, opts)
    %REGISTERWORLDFRAME Register the Kinect camera to the UWB anchor frame.
    %
    % UWB-repo port (2026-07-02) of MMS vision core/registerWorldFrame.m —
    % algorithm unchanged; saves into this folder's calibration_data/.
    %
    % Solves the rigid transform (R, t) mapping camera-frame coordinates to a
    % surveyed testbed frame (the UWB anchor frame from anchors.json:
    % origin = anchor 1, +X toward anchor 2), by detecting an AprilTag placed
    % at K >= 3 known floor points. Replaces the plumb-camera (nadir)
    % assumption, which cannot see camera tilt, mount-height error, or the
    % offset/rotation between image axes and the testbed axes.
    %
    % PROCEDURE
    %   1. Survey K >= 4 points on the testbed floor in the SAME frame the UWB
    %      anchors are expressed in (tape-measure from the anchor-1 corner,
    %      the same way the anchors were surveyed). Spread them across the
    %      floor - corners + center is ideal. Record the coordinates of the
    %      TAG CENTER when the tag lies on each mark (z = tag stand/board
    %      thickness, usually ~0).
    %   2. Run: reg = registerWorldFrame(worldPoints);
    %      where worldPoints is Kx3 [x y z] in meters.
    %   3. Place the tag flat on point 1, press ENTER; repeat for each point.
    %   4. Review the residual report. RMSE should be well under your target
    %      accuracy (aim < 0.02 m). The transform is saved to
    %      calibration_data/world_registration.mat and picked up automatically
    %      by visionSystemConfig() / transformPoseToWorld().
    %
    % Inputs:
    %   worldPoints - Kx3 surveyed tag-center coordinates (m), K >= 3,
    %                 non-collinear
    %   opts        - optional struct:
    %                   .framesPerPoint  frames averaged per point (default 40)
    %                   .tagID           AprilTag id to use (default: base tag)
    %                   .saveFile        output .mat path (default:
    %                                    calibration_data/world_registration.mat)
    %                   .dryRun          if true, do not save (default false)
    %
    % Output:
    %   registration struct: R_cam2world, t_cam2world, rmse_m, residuals_m,
    %   worldPoints, camPoints, registeredDate.

    if nargin < 1 || size(worldPoints, 2) ~= 3 || size(worldPoints, 1) < 3
        error('registerWorldFrame:badInput', ...
              'worldPoints must be a Kx3 matrix with K >= 3 surveyed points.');
    end
    if nargin < 2, opts = struct(); end
    if ~isfield(opts, 'framesPerPoint'), opts.framesPerPoint = 40; end
    if ~isfield(opts, 'dryRun'), opts.dryRun = false; end

    config = visionSystemConfig();
    if ~isfield(opts, 'tagID'), opts.tagID = config.apriltag.base.id; end
    if ~isfield(opts, 'saveFile')
        opts.saveFile = fullfile(fileparts(mfilename('fullpath')), ...
                                 'calibration_data', 'world_registration.mat');
    end

    % Collinearity check (rank of centered points)
    K = size(worldPoints, 1);
    centered = worldPoints - mean(worldPoints, 1);
    if rank(centered, 1e-6) < 2
        error('registerWorldFrame:collinear', ...
              'Surveyed points are collinear - spread them across the floor.');
    end

    %% Capture camera-frame tag positions at each surveyed point
    kinectObj = imaq.VideoDevice(config.camera.deviceName, ...
                                 config.camera.deviceID, ...
                                 config.camera.colorFormat);
    kinectObj.ReturnedColorSpace = config.camera.returnedColorSpace;
    cleanupObj = onCleanup(@() release(kinectObj));

    camPoints = nan(K, 3);
    fprintf('=== World-frame registration: %d points, tag ID %d ===\n', K, opts.tagID);
    for k = 1:K
        fprintf('\nPoint %d/%d: world (%.3f, %.3f, %.3f) m\n', ...
                k, K, worldPoints(k, 1), worldPoints(k, 2), worldPoints(k, 3));
        input('  Place the tag flat on this mark, then press ENTER...', 's');

        samples = nan(opts.framesPerPoint, 3);
        got = 0;
        attempts = 0;
        while got < opts.framesPerPoint && attempts < opts.framesPerPoint * 4
            attempts = attempts + 1;
            rgbFrame = step(kinectObj);
            rgbFrame = rgbFrame(:, :, [3, 2, 1]);   % BGR -> RGB
            grayFrame = fliplr(rgb2gray(rgbFrame)); % same convention as trackers
            [ids, ~, poses] = readAprilTag(grayFrame, ...
                                           config.apriltag.base.family, ...
                                           config.camera.intrinsics, ...
                                           config.apriltag.base.size);
            idx = find(ids == opts.tagID, 1);
            if ~isempty(idx)
                got = got + 1;
                samples(got, :) = poses(idx).Translation;
            end
        end
        if got < max(5, opts.framesPerPoint / 4)
            error('registerWorldFrame:noDetection', ...
                  'Point %d: only %d/%d detections - check lighting/visibility.', ...
                  k, got, opts.framesPerPoint);
        end
        camPoints(k, :) = median(samples(1:got, :), 1);
        spread = std(samples(1:got, :), 0, 1);
        fprintf('  Captured %d frames. cam-frame (%.3f, %.3f, %.3f) m, std [%.1f %.1f %.1f] mm\n', ...
                got, camPoints(k, :), spread * 1000);
    end

    %% Kabsch: R, t minimizing || R*cam + t - world ||
    muC = mean(camPoints, 1);
    muW = mean(worldPoints, 1);
    H = (camPoints - muC)' * (worldPoints - muW);
    [U, ~, V] = svd(H);
    D = eye(3);
    D(3, 3) = sign(det(V * U'));
    R = V * D * U';
    t = muW' - R * muC';

    if det(V * U') < 0
        warning(['registerWorldFrame:reflection - the best-fit transform ' ...
                 'wanted a reflection (det < 0). This means the image-mirroring ' ...
                 'convention (fliplr) is inconsistent with the survey. Check ' ...
                 'that world X/Y axes are not swapped/mirrored, and that the ' ...
                 'same fliplr convention is used everywhere. Residuals below ' ...
                 'are for the best PROPER rotation and may be poor.']);
    end

    %% Residual report
    mapped = (R * camPoints' + t)';
    residuals = mapped - worldPoints;
    perPoint = sqrt(sum(residuals.^2, 2));
    rmse = sqrt(mean(perPoint.^2));

    fprintf('\n=== Registration result ===\n');
    fprintf('R_cam2world =\n'); disp(R);
    fprintf('t_cam2world = [%.4f, %.4f, %.4f] m\n', t);
    tiltDeg = acosd(max(-1, min(1, abs(R(3, 3)))));
    fprintf('Implied camera height: %.3f m | optical-axis tilt from plumb: %.2f deg\n', ...
            t(3), tiltDeg);
    fprintf('%-6s %-28s %-28s %-10s\n', 'Point', 'World (m)', 'Mapped (m)', 'Err (mm)');
    for k = 1:K
        fprintf('%-6d (%6.3f, %6.3f, %6.3f)    (%6.3f, %6.3f, %6.3f)    %7.1f\n', ...
                k, worldPoints(k, :), mapped(k, :), perPoint(k) * 1000);
    end
    fprintf('RMSE: %.1f mm | max: %.1f mm\n', rmse * 1000, max(perPoint) * 1000);
    if rmse > 0.03
        warning(['registerWorldFrame:highResidual - RMSE %.1f mm is high for a ' ...
                 'cm-level ground-truth target. Re-survey the points, check the ' ...
                 'tag lies flat, and verify intrinsics.'], rmse * 1000);
    end

    %% Save
    registration = struct('R_cam2world', R, 't_cam2world', t, ...
                          'rmse_m', rmse, 'residuals_m', residuals, ...
                          'worldPoints', worldPoints, 'camPoints', camPoints, ...
                          'framesPerPoint', opts.framesPerPoint, ...
                          'tagID', opts.tagID, ...
                          'registeredDate', datestr(now, 'yyyy-mm-dd HH:MM:SS')); %#ok<TNOW1,DATST>
    if ~opts.dryRun
        saveDir = fileparts(opts.saveFile);
        if ~isempty(saveDir) && ~exist(saveDir, 'dir')
            mkdir(saveDir);
        end
        R_cam2world = R; t_cam2world = t; rmse_m = rmse; residuals_m = residuals; %#ok<NASGU>
        registeredDate = registration.registeredDate; %#ok<NASGU>
        save(opts.saveFile, 'R_cam2world', 't_cam2world', 'rmse_m', ...
             'residuals_m', 'worldPoints', 'camPoints', 'registeredDate');
        fprintf('Saved to %s\nvisionSystemConfig() will now use this transform automatically.\n', ...
                opts.saveFile);
    else
        fprintf('Dry run - nothing saved.\n');
    end
end
