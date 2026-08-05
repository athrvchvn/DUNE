function A = clickAnchorsWorld(anchorZ, opts)
    %CLICKANCHORSWORLD Click each UWB anchor antenna -> world coords -> anchors.json.
    %
    %   clickAnchorsWorld            % z = 0.24 m, anchors 1..5
    %   clickAnchorsWorld(0.24, struct('dryRun', true))
    %
    % Takes a fresh Kinect snapshot and asks for ONE CLICK on each anchor's
    % antenna. Each click is converted to world x,y on the z = anchorZ plane
    % using the SAVED registration (world_registration.mat) - this tool never
    % redefines the world frame, so anchors can be re-placed and re-clicked
    % any number of times and all coordinates stay in the established frame.
    % (Do NOT use measureAnchorsFromCamera for this - it re-anchors the frame
    % to a1/a2.)
    %
    % After the clicks it prints the positions and the pairwise distance
    % table (spot-check against a tape measure if unsure), waits for ENTER,
    % then writes matlab/config/anchors.json with a timestamped backup of the
    % previous file.
    %
    % Inputs:
    %   anchorZ - anchor antenna height above the floor (m), default 0.24
    %   opts    - optional struct fields:
    %       .anchorIds  which ids to click, in order (default [1 2 3 4 5])
    %       .outFile    default matlab/config/anchors.json
    %       .snapshot   HxWx3 image to use instead of a live Kinect grab
    %                   (already in the pipeline's fliplr'd convention)
    %       .dryRun     print everything, write nothing (default false)
    %
    % Output: the anchors struct that was written (dune.loadAnchors format).

    if nargin < 1 || isempty(anchorZ), anchorZ = 0.24; end
    if nargin < 2, opts = struct(); end
    if ~isfield(opts, 'anchorIds'), opts.anchorIds = [1 2 3 4 5]; end
    if ~isfield(opts, 'dryRun'),    opts.dryRun = false; end
    if ~isfield(opts, 'outFile')
        opts.outFile = fullfile(fileparts(mfilename('fullpath')), ...
                                '..', 'config', 'anchors.json');
    end

    config = visionSystemConfig();
    if ~config.extrinsics.available
        error('clickAnchorsWorld:noRegistration', ...
              'No world registration found - run registerWorldFrameClicks first.');
    end
    R = config.extrinsics.R;
    t = config.extrinsics.t(:);
    fprintf('Using registration: %s (RMSE %.1f mm)\n', ...
            config.extrinsics.file, config.extrinsics.rmse_m * 1000);

    %% Snapshot (same fliplr convention as the rest of the pipeline)
    if isfield(opts, 'snapshot') && ~isempty(opts.snapshot)
        rgbFrame = opts.snapshot;
    else
        kinectObj = imaq.VideoDevice(config.camera.deviceName, ...
                                     config.camera.deviceID, ...
                                     config.camera.colorFormat);
        kinectObj.ReturnedColorSpace = config.camera.returnedColorSpace;
        cleanupObj = onCleanup(@() release(kinectObj));
        rgbFrame = step(kinectObj);
        rgbFrame = rgbFrame(:, :, [3, 2, 1]);
        rgbFrame = fliplr(rgbFrame);
    end

    fx = config.camera.focalLength(1);  fy = config.camera.focalLength(2);
    cx = config.camera.principalPoint(1); cy = config.camera.principalPoint(2);

    ids = opts.anchorIds(:);
    nA  = numel(ids);
    fig = figure('Name', 'Click anchor antennas', 'Position', [50, 50, 1500, 850]);
    while true
        clf(fig);
        imshow(rgbFrame);
        hold on;
        title('Click each anchor ANTENNA tip (zoom first if needed)', 'FontSize', 12);
        pos = nan(nA, 2);
        for k = 1:nA
            xlabel(sprintf('>>> Click anchor A%d antenna (z = %.2f m)   [%d/%d]', ...
                           ids(k), anchorZ, k, nA), ...
                   'FontSize', 13, 'FontWeight', 'bold', 'Color', 'red');
            [u, v] = ginput(1);
            plot(u, v, 'g+', 'MarkerSize', 14, 'LineWidth', 2);
            % Ray through the (undistorted) pixel, intersected with z = anchorZ
            uvU = undistortPts([u, v], config.camcal);
            ray = [(uvU(1) - cx) / fx; (uvU(2) - cy) / fy; 1];
            dirW = R * ray;
            s = (anchorZ - t(3)) / dirW(3);
            pw = R * (s * ray) + t;
            pos(k, :) = pw(1:2)';
            text(u + 12, v, sprintf('A%d (%.2f, %.2f)', ids(k), pos(k, :)), ...
                 'Color', 'green', 'FontSize', 11, 'FontWeight', 'bold');
        end
        xlabel('Done.', 'Color', 'black');

        %% Report
        fprintf('\n=== Clicked anchor positions (z = %.2f m plane) ===\n', anchorZ);
        for k = 1:nA
            fprintf('  A%d : (%8.3f, %8.3f)\n', ids(k), pos(k, :));
        end
        fprintf('\nPairwise distances (m) - tape-check any you are unsure about:\n');
        fprintf('%6s', '');
        fprintf('%9s', compose("A%d", ids));
        fprintf('\n');
        for i = 1:nA
            fprintf('%6s', sprintf('A%d', ids(i)));
            for j = 1:nA
                if j <= i, fprintf('%9s', ''); else
                    fprintf('%9.3f', norm(pos(i, :) - pos(j, :)));
                end
            end
            fprintf('\n');
        end

        ans_ = input('\nAccept and write anchors.json? (ENTER = yes, r = redo clicks): ', 's');
        if ~strcmpi(strtrim(ans_), 'r')
            break;
        end
    end

    %% Build anchors struct + write
    anchorsArr = struct('id', {}, 'x', {}, 'y', {}, 'z', {});
    for k = 1:nA
        anchorsArr(k) = struct('id', ids(k), ...
                               'x', round(pos(k, 1), 4), ...
                               'y', round(pos(k, 2), 4), ...
                               'z', anchorZ);
    end
    out = struct('dim', 2, ...
                 'bounds', [min(pos(:,1)) - 0.5, max(pos(:,1)) + 0.5, ...
                            min(pos(:,2)) - 0.5, max(pos(:,2)) + 0.5, 0.0, 3.0], ...
                 'anchors', anchorsArr, ...
                 'layout', sprintf('clicked_kinect_%s_z%.2f', ...
                                   datestr(now, 'yyyy-mm-dd'), anchorZ)); %#ok<TNOW1,DATST>

    if opts.dryRun
        fprintf('Dry run - nothing saved.\n');
    else
        if exist(opts.outFile, 'file')
            bak = sprintf('%s.bak_%s', opts.outFile, ...
                          datestr(now, 'yyyymmdd_HHMMSS')); %#ok<TNOW1,DATST>
            copyfile(opts.outFile, bak);
            fprintf('Previous anchors.json backed up to %s\n', bak);
        end
        fh = fopen(opts.outFile, 'w');
        fwrite(fh, jsonencode(out, 'PrettyPrint', true));
        fclose(fh);
        fprintf('Written to %s\n', opts.outFile);
    end

    A = struct('ids', ids, 'pos', [pos, repmat(anchorZ, nA, 1)], ...
               'bounds', out.bounds, 'dim', 2, 'layout', out.layout, ...
               'file', opts.outFile);
end
