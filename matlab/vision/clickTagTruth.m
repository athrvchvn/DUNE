function truth = clickTagTruth(tagZ, opts)
    %CLICKTAGTRUTH Click the parked UWB tags -> ground-truth ranges JSON.
    %
    % Uses the SAVED camera registration (run registerWorldFrameClicks or
    % registerWorldFrame first) to turn two clicks on a fresh Kinect snapshot
    % into world positions of the UWB tag antennas, computes the true range
    % from each tag to every anchor in anchors.json, and writes
    % matlab/config/tag_truth.json for the Python side:
    %
    %   python tag_selfcalib.py auto --tag-id 240
    %
    % which closes the loop: live ranges vs these truths -> mean bias over all
    % anchors -> SETMYDELAY correction pushed to the tag (NVS-persisted).
    %
    % IMPORTANT: the unit must NOT move between this click and the Python
    % auto-calibration run. Re-run this any time the unit is re-parked.
    %
    % Inputs:
    %   tagZ - height of the UWB tag antennas above the floor (m)
    %   opts - optional. Either a vector of tag ids to click, e.g.
    %              clickTagTruth(0.22, 240)         % single tag
    %          or a struct: .uwbTagIds (default [240 241]),
    %          .anchorsFile, .outFile, .dryRun
    %
    % Output: the truth struct that was written.

    if nargin < 1 || ~isscalar(tagZ)
        error('clickTagTruth:badInput', ...
              'Give the tag antenna height above the floor in meters, e.g. clickTagTruth(0.22)');
    end
    if nargin < 2, opts = struct(); end
    if isnumeric(opts), opts = struct('uwbTagIds', opts); end
    if ~isfield(opts, 'uwbTagIds'), opts.uwbTagIds = [240, 241]; end
    if ~isfield(opts, 'dryRun'), opts.dryRun = false; end
    if ~isfield(opts, 'anchorsFile')
        opts.anchorsFile = fullfile(fileparts(mfilename('fullpath')), ...
                                    '..', 'config', 'anchors.json');
    end
    if ~isfield(opts, 'outFile')
        opts.outFile = fullfile(fileparts(mfilename('fullpath')), ...
                                '..', 'config', 'tag_truth.json');
    end

    config = visionSystemConfig();
    if ~config.extrinsics.available
        error('clickTagTruth:noRegistration', ...
              'No world registration found - run registerWorldFrameClicks first.');
    end
    R = config.extrinsics.R;
    t = config.extrinsics.t(:);
    fprintf('Using registration: %s (RMSE %.1f mm)\n', ...
            config.extrinsics.file, config.extrinsics.rmse_m * 1000);

    %% Anchors
    aj = jsondecode(fileread(opts.anchorsFile));
    anchorIds = arrayfun(@(a) a.id, aj.anchors);
    anchorPos = [arrayfun(@(a) a.x, aj.anchors), ...
                 arrayfun(@(a) a.y, aj.anchors), ...
                 arrayfun(@(a) a.z, aj.anchors)];

    %% Snapshot (same fliplr convention as the trackers)
    kinectObj = imaq.VideoDevice(config.camera.deviceName, ...
                                 config.camera.deviceID, ...
                                 config.camera.colorFormat);
    kinectObj.ReturnedColorSpace = config.camera.returnedColorSpace;
    cleanupObj = onCleanup(@() release(kinectObj));
    rgbFrame = step(kinectObj);
    rgbFrame = rgbFrame(:, :, [3, 2, 1]);
    rgbFrame = fliplr(rgbFrame);

    fx = config.camera.focalLength(1);  fy = config.camera.focalLength(2);
    cx = config.camera.principalPoint(1); cy = config.camera.principalPoint(2);

    nT = numel(opts.uwbTagIds);
    fig = figure('Name', 'Click UWB tag antennas', 'Position', [50, 50, 1500, 850]);
    while true
        clf(fig);
        imshow(rgbFrame);
        hold on;
        title('Click each UWB tag ANTENNA (zoom first if needed)', 'FontSize', 12);
        tagPix = nan(nT, 2);
        tagXY  = nan(nT, 2);
        for k = 1:nT
            xlabel(sprintf('>>> Click UWB tag 0x%02X antenna (z = %.2f m)   [%d/%d]', ...
                           opts.uwbTagIds(k), tagZ, k, nT), ...
                   'FontSize', 13, 'FontWeight', 'bold', 'Color', 'red');
            [u, v] = ginput(1);
            tagPix(k, :) = [u, v];
            plot(u, v, 'g+', 'MarkerSize', 14, 'LineWidth', 2);
            text(u + 12, v, sprintf('0x%02X', opts.uwbTagIds(k)), 'Color', 'green', ...
                 'FontSize', 12, 'FontWeight', 'bold');
            % Ray through the (undistorted) pixel, intersected with z = tagZ
            uvU = undistortPts([u, v], config.camcal);
            ray = [(uvU(1) - cx) / fx; (uvU(2) - cy) / fy; 1];
            dirW = R * ray;
            s = (tagZ - t(3)) / dirW(3);
            pw = R * (s * ray) + t;
            tagXY(k, :) = pw(1:2)';
        end
        xlabel('Done.', 'Color', 'black');
        ans_ = input('Accept clicks? (ENTER = yes, r = redo): ', 's');
        if ~strcmpi(strtrim(ans_), 'r')
            break;
        end
    end

    %% Truth table
    fprintf('\n=== Tag truth (z = %.2f m plane) ===\n', tagZ);
    tags = struct('id', {}, 'x', {}, 'y', {}, 'z', {}, 'true_ranges_m', {});
    for k = 1:nT
        p = [tagXY(k, :), tagZ];
        rr = struct();
        fprintf('  tag 0x%02X : world (%.3f, %.3f)\n', opts.uwbTagIds(k), tagXY(k, :));
        for a = 1:numel(anchorIds)
            d = norm(p - anchorPos(a, :));
            rr.(sprintf('a%d', anchorIds(a))) = round(d, 4);
            fprintf('      -> anchor 0x%02X : %.3f m\n', anchorIds(a), d);
        end
        tags(k) = struct('id', opts.uwbTagIds(k), ...
                         'x', round(tagXY(k, 1), 4), 'y', round(tagXY(k, 2), 4), ...
                         'z', tagZ, 'true_ranges_m', rr);
    end
    if nT == 2
        fprintf('  inter-tag baseline: %.3f m (tape says 0.480)\n', ...
                norm(tagXY(1, :) - tagXY(2, :)));
    end

    truth = struct('version', 1, ...
                   'timestamp', datestr(now, 'yyyy-mm-ddTHH:MM:SS'), ... %#ok<TNOW1,DATST>
                   'tag_z', tagZ, ...
                   'registration_rmse_m', config.extrinsics.rmse_m, ...
                   'tags', tags);

    if ~opts.dryRun
        fh = fopen(opts.outFile, 'w');
        fwrite(fh, jsonencode(truth, 'PrettyPrint', true));
        fclose(fh);
        fprintf('\nWritten to %s\n', opts.outFile);
        fprintf('Now (WITHOUT moving the unit) run e.g.:\n');
        fprintf('  python tag_selfcalib.py auto --tag-id 240\n');
    else
        fprintf('Dry run - nothing saved.\n');
    end
end
