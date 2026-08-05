function aprilTagMinSizeTest(tagFamily)
    %APRILTAGMINSIZETEST Find the smallest AprilTag the Kinect setup can detect.
    %
    % Lay your printed tags (various sizes, ideally DISTINCT IDs) on the floor
    % in view of the overhead Kinect. The script live-detects every tag,
    % overlays its family + ID + side length in pixels + estimated physical
    % size, and tracks a per-tag detection rate so flickery (marginal) tags
    % stand out from solid ones. Quit to get a summary table sorted by size —
    % the smallest tag with a high detection rate is your answer.
    %
    % Detection uses family only (no intrinsics/size), because the tags on
    % the floor have different physical sizes — pose estimation with a single
    % size constant would be wrong for all but one of them. Physical size is
    % estimated from the nadir scale: size ≈ px_side * height / fx, valid for
    % tags lying flat on the floor.
    %
    %   Usage:
    %     aprilTagMinSizeTest()                        % config family (tag36h11)
    %     aprilTagMinSizeTest("tag16h5")               % a single other family
    %     aprilTagMinSizeTest(["tag36h11","tag16h5"])  % several at once (slower)
    %     aprilTagMinSizeTest("all")                   % every family (slowest)
    %
    %   Supported families (readAprilTag): tag16h5, tag25h9, tag36h10,
    %   tag36h11, tagCircle21h7, tagCircle49h12, tagCustom48h12,
    %   tagStandard41h12, tagStandard52h13.
    %
    %   CAUTION tag16h5: only 30 codes and hamming distance 5 — it is prone
    %   to false positives (phantom detections on floor texture/clutter).
    %   Prefer running it as an explicit single family, and distrust IDs you
    %   did not print, especially at low detection rates.
    %
    %   Controls:
    %     'c' = Clear stats (press after you finish arranging the tags!)
    %     's' = Save snapshot image to matlab/vision/logs/
    %     'q' = Quit, print summary, save CSV

    %% Configuration (single source of truth: visionSystemConfig.m)
    config = visionSystemConfig();
    if nargin < 1 || isempty(tagFamily)
        tagFamily = config.apriltag.base.family;   % 'tag36h11'
    end
    tagFamily = string(tagFamily);
    fx = config.camera.focalLength(1);
    camHeight = config.camera.height;          % m above floor (nadir estimate)
    pxPerCm = fx / camHeight / 100;            % image scale at the floor plane

    windowLen = 60;                            % rolling detection-rate window (frames)

    fprintf('==============================================\n');
    fprintf('AprilTag Minimum-Size Detection Test\n');
    fprintf('==============================================\n');
    fprintf('Families: %s | fx = %.0f px | camera height = %.2f m\n', ...
            strjoin(tagFamily, ', '), fx, camHeight);
    fprintf('Floor-plane scale: %.2f px/cm (1 cm on the floor = %.2f px)\n', ...
            pxPerCm, pxPerCm);
    fprintf('Rule of thumb (~2 px per module incl. border + quiet zone):\n');
    for f = tagFamily(:)'
        nMod = familyModules(f);
        if ~isnan(nMod)
            fprintf('  %-18s %2d modules -> limit near %2.0f px = ~%.1f cm\n', ...
                    f, nMod, 2 * nMod, 2 * nMod / pxPerCm);
        end
    end
    fprintf('\nControls: ''c'' clear stats | ''s'' snapshot | ''q'' quit + summary\n\n');

    %% Initialize Kinect
    fprintf('Initializing Kinect v2...\n');
    kinectObj = imaq.VideoDevice(config.camera.deviceName, ...
                                 config.camera.deviceID, ...
                                 config.camera.colorFormat);
    kinectObj.ReturnedColorSpace = config.camera.returnedColorSpace;
    fprintf('[OK] Kinect initialized\n\n');

    %% Stats (keyed by "family#id" so IDs can repeat across families)
    % stats(key) = struct: family, id, count, sumPxSide, lastPxSide, lastSeenFrame
    stats = containers.Map('KeyType', 'char', 'ValueType', 'any');
    frameCount = 0;                        % frames since last stats clear
    ringBuf = cell(windowLen, 1);          % last N frames' detected key lists

    %% Create Figure
    userCommand = '';
    fig = figure('Position', [50, 50, 1600, 900], ...
                 'Name', 'AprilTag Min-Size Test', ...
                 'KeyPressFcn', @(src, evt) keyPressCallback(evt));

    %% Main Loop
    try
        while ishandle(fig)
            % Capture frame (same pipeline as the other vision scripts:
            % BGR->RGB swap + un-mirror the Kinect color stream)
            rgbFrame = step(kinectObj);
            rgbFrame = rgbFrame(:,:,[3,2,1]);
            rgbFrame = fliplr(rgbFrame);
            grayFrame = rgb2gray(rgbFrame);

            % Detect (IDs + pixel corners only — mixed physical sizes)
            [ids, locs, detFams] = readAprilTag(grayFrame, tagFamily);
            detFams = string(detFams);

            % Update stats
            frameCount = frameCount + 1;
            frameKeys = strings(1, length(ids));
            for i = 1:length(ids)
                frameKeys(i) = sprintf('%s#%d', detFams(i), ids(i));
                pxSide = meanSidePx(locs(:,:,i));
                if isKey(stats, char(frameKeys(i)))
                    s = stats(char(frameKeys(i)));
                else
                    s = struct('family', detFams(i), 'id', double(ids(i)), ...
                               'count', 0, 'sumPxSide', 0, ...
                               'lastPxSide', 0, 'lastSeenFrame', 0);
                end
                s.count = s.count + 1;
                s.sumPxSide = s.sumPxSide + pxSide;
                s.lastPxSide = pxSide;
                s.lastSeenFrame = frameCount;
                stats(char(frameKeys(i))) = s;
            end
            ringBuf{mod(frameCount - 1, windowLen) + 1} = frameKeys;

            % Visualize
            visualizeMinSize(fig, rgbFrame, ids, locs, detFams, frameKeys, ...
                             stats, ringBuf, frameCount, windowLen, pxPerCm);

            % Handle user input
            if ~isempty(userCommand)
                switch userCommand
                    case 'c'
                        stats = containers.Map('KeyType', 'char', 'ValueType', 'any');
                        frameCount = 0;
                        ringBuf = cell(windowLen, 1);
                        fprintf('[CLEARED] Stats reset — measuring starts now.\n');
                    case 's'
                        snapFile = saveSnapshot(rgbFrame, ids, locs, detFams);
                        fprintf('[SAVED] Snapshot: %s\n', snapFile);
                    case 'q'
                        fprintf('\n[QUIT] Stopping test...\n');
                        break;
                end
                userCommand = '';
            end

            pause(0.02);
        end
    catch ME
        if ~strcmp(ME.identifier, 'MATLAB:interruption')
            fprintf('\n[ERROR] %s\n', ME.message);
        end
    end

    %% Summary + save
    printSummary(stats, frameCount, pxPerCm, tagFamily, camHeight);
    saveSummaryCsv(stats, frameCount, pxPerCm);

    %% Cleanup
    release(kinectObj);
    if ishandle(fig); close(fig); end
    fprintf('\n[OK] Test completed!\n');

    function keyPressCallback(evt)
        userCommand = evt.Key;
    end
end

function nMod = familyModules(family)
    %FAMILYMODULES Total tag width in modules, incl. border + quiet zone.
    switch char(family)
        case 'tag16h5',           nMod = 8;
        case 'tag25h9',           nMod = 9;
        case {'tag36h10', 'tag36h11'}, nMod = 10;
        case 'tagCircle21h7',     nMod = 9;
        case 'tagCircle49h12',    nMod = 11;
        case 'tagCustom48h12',    nMod = 10;
        case 'tagStandard41h12',  nMod = 9;
        case 'tagStandard52h13',  nMod = 10;
        otherwise,                nMod = NaN;   % 'all' or unknown
    end
end

function pxSide = meanSidePx(corners)
    %MEANSIDEPX Mean side length (px) of a 4x2 corner quad.
    d = corners - corners([2 3 4 1], :);
    pxSide = mean(sqrt(sum(d.^2, 2)));
end

function rate = rollingRate(key, ringBuf, frameCount, windowLen)
    %ROLLINGRATE Fraction of the last min(frameCount,windowLen) frames with this key.
    n = min(frameCount, windowLen);
    if n == 0; rate = 0; return; end
    hits = 0;
    for k = 1:n
        if any(ringBuf{k} == key); hits = hits + 1; end
    end
    rate = hits / n;
end

function shortName = famShort(family)
    %FAMSHORT Compact family label ('tag36h11' -> '36h11').
    shortName = regexprep(char(family), '^tag', '');
end

function visualizeMinSize(fig, rgbFrame, ids, locs, detFams, frameKeys, ...
                          stats, ringBuf, frameCount, windowLen, pxPerCm)
    %VISUALIZEMINSIZE Camera view with per-tag overlays + live stats panel.

    if ~ishandle(fig) || ~isvalid(fig); return; end
    figure(fig);

    %% Left: Camera View
    subplot(1, 2, 1);
    imshow(rgbFrame);
    hold on;

    for i = 1:length(ids)
        corners = locs(:,:,i);
        pxSide = meanSidePx(corners);
        estCm = pxSide / pxPerCm;
        rate = rollingRate(frameKeys(i), ringBuf, frameCount, windowLen);

        % Green = solid detection, yellow = marginal (flickering)
        if rate >= 0.9
            col = 'g';
        else
            col = 'y';
        end
        plot([corners(:,1); corners(1,1)], [corners(:,2); corners(1,2)], ...
             [col '-'], 'LineWidth', 3);
        cx = mean(corners(:,1));
        cy = mean(corners(:,2));
        text(cx, cy - pxSide/2 - 18, ...
             sprintf('%s ID %d  %.0fpx  ~%.1fcm  %d%%', ...
                     famShort(detFams(i)), ids(i), pxSide, estCm, round(rate*100)), ...
             'Color', col, 'FontSize', 11, 'FontWeight', 'bold', ...
             'HorizontalAlignment', 'center', 'BackgroundColor', [0 0 0]);
    end

    if ~isempty(ids)
        rectangle('Position', [20, 20, 260, 40], 'FaceColor', [0, 0.5, 0, 0.8], ...
                  'EdgeColor', 'white', 'LineWidth', 2);
        text(150, 40, sprintf('%d TAG(S) VISIBLE', length(ids)), ...
             'Color', 'white', 'FontSize', 14, 'FontWeight', 'bold', ...
             'HorizontalAlignment', 'center');
    else
        rectangle('Position', [20, 20, 260, 40], 'FaceColor', [0.7, 0, 0, 0.8], ...
                  'EdgeColor', 'white', 'LineWidth', 2);
        text(150, 40, 'NO TAG', 'Color', 'white', 'FontSize', 14, ...
             'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    end
    hold off;
    title(sprintf('Live detection — frame %d (stats window %d)', ...
                  frameCount, windowLen), 'FontSize', 12, 'FontWeight', 'bold');

    %% Right: Stats Panel
    subplot(1, 2, 2);
    cla;
    axis off;

    text(0.02, 0.97, 'DETECTED TAGS (since last clear)', ...
         'FontSize', 13, 'FontWeight', 'bold', 'Color', 'blue');
    text(0.02, 0.91, sprintf('%-8s %-5s %-9s %-9s %-11s %-10s', ...
         'family', 'ID', 'px side', 'est size', 'rate(now)', 'rate(all)'), ...
         'FontSize', 10, 'FontWeight', 'bold', 'FontName', 'FixedWidth');

    % Sort by estimated size, largest first
    keyList = keys(stats);
    n = numel(keyList);
    meanPx = zeros(1, n);
    for k = 1:n
        s = stats(keyList{k});
        meanPx(k) = s.sumPxSide / s.count;
    end
    [~, order] = sort(meanPx, 'descend');

    yPos = 0.86;
    for k = order
        s = stats(keyList{k});
        estCm = meanPx(k) / pxPerCm;
        rateNow = rollingRate(string(keyList{k}), ringBuf, frameCount, windowLen);
        rateAll = s.count / max(frameCount, 1);
        if rateNow >= 0.9
            col = [0, 0.5, 0];
        elseif rateNow > 0
            col = [0.8, 0.5, 0];
        else
            col = [0.6, 0.6, 0.6];   % seen before, not currently visible
        end
        text(0.02, yPos, sprintf('%-8s %-5d %6.0f px %6.1f cm %8d%% %8d%%', ...
             famShort(s.family), s.id, meanPx(k), estCm, ...
             round(rateNow*100), round(rateAll*100)), ...
             'FontSize', 10, 'FontName', 'FixedWidth', 'Color', col);
        yPos = yPos - 0.045;
        if yPos < 0.30; break; end   % panel full
    end

    text(0.02, 0.22, sprintf('Floor scale: %.2f px/cm', pxPerCm), ...
         'FontSize', 10, 'Color', [0.3, 0.3, 0.3]);
    text(0.02, 0.16, 'Controls:', 'FontSize', 11, 'FontWeight', 'bold');
    text(0.06, 0.11, '''c'' = clear stats (after arranging tags)', 'FontSize', 10);
    text(0.06, 0.07, '''s'' = save snapshot', 'FontSize', 10);
    text(0.06, 0.03, '''q'' = quit + summary + CSV', 'FontSize', 10);

    xlim([0, 1]);
    ylim([0, 1]);
    drawnow;
end

function printSummary(stats, frameCount, pxPerCm, tagFamily, camHeight)
    %PRINTSUMMARY Final table sorted smallest-first + verdict.

    fprintf('\n==============================================\n');
    fprintf('MINIMUM TAG SIZE — SUMMARY (%s, %d frames)\n', ...
            strjoin(tagFamily, ', '), frameCount);
    fprintf('==============================================\n');

    if isempty(stats) || frameCount == 0
        fprintf('No tags detected.\n');
        return;
    end

    keyList = keys(stats);
    n = numel(keyList);
    meanPx = zeros(1, n); rateAll = zeros(1, n);
    for k = 1:n
        s = stats(keyList{k});
        meanPx(k) = s.sumPxSide / s.count;
        rateAll(k) = s.count / frameCount;
    end
    [~, order] = sort(meanPx, 'ascend');   % smallest first

    fprintf('%-10s %-6s %-10s %-12s %-10s %s\n', ...
            'family', 'ID', 'px side', 'est size', 'det rate', 'verdict');
    fprintf('--------------------------------------------------------------\n');
    smallestReliable = NaN;
    for k = order
        s = stats(keyList{k});
        estCm = meanPx(k) / pxPerCm;
        if rateAll(k) >= 0.95
            verdict = 'RELIABLE';
            if isnan(smallestReliable); smallestReliable = estCm; end
        elseif rateAll(k) >= 0.5
            verdict = 'marginal';
        else
            verdict = 'unreliable';
        end
        fprintf('%-10s %-6d %7.0f px %8.1f cm %8.0f%%  %s\n', ...
                famShort(s.family), s.id, meanPx(k), estCm, ...
                rateAll(k)*100, verdict);
    end
    fprintf('--------------------------------------------------------------\n');
    if ~isnan(smallestReliable)
        fprintf(['Smallest RELIABLY detected tag: ~%.1f cm ' ...
                 '(>=95%% detection rate at %.2f m camera height)\n'], ...
                smallestReliable, camHeight);
    else
        fprintf('No tag reached a 95%% detection rate.\n');
    end
    fprintf(['NOTE: est size assumes the tag lies flat on the floor; ' ...
             'verify against the ruler.\n']);
    fprintf(['NOTE: tag16h5 detections you did not print are likely FALSE ' ...
             'POSITIVES (weak family).\n']);
end

function saveSummaryCsv(stats, frameCount, pxPerCm)
    %SAVESUMMARYCSV Save the per-tag summary to matlab/vision/logs/.

    if isempty(stats) || frameCount == 0; return; end

    logDir = fullfile(fileparts(mfilename('fullpath')), 'logs');
    if ~exist(logDir, 'dir'); mkdir(logDir); end
    timeStr = datestr(datetime('now'), 'yyyy-mm-dd_HH-MM-SS');
    csvFile = fullfile(logDir, sprintf('apriltag_minsize_%s.csv', timeStr));

    keyList = keys(stats);
    n = numel(keyList);
    fams = strings(n, 1);
    rows = zeros(n, 5);
    for k = 1:n
        s = stats(keyList{k});
        mpx = s.sumPxSide / s.count;
        fams(k) = s.family;
        rows(k, :) = [s.id, mpx, mpx / pxPerCm, s.count, s.count / frameCount];
    end
    [~, order] = sort(rows(:, 3));   % by estimated size
    T = table(fams(order), rows(order, 1), rows(order, 2), rows(order, 3), ...
              rows(order, 4), rows(order, 5), 'VariableNames', ...
        {'Family', 'TagID', 'MeanSide_px', 'EstSize_cm', 'Detections', 'DetectionRate'});
    writetable(T, csvFile);
    fprintf('[SAVED] CSV: %s\n', csvFile);
end

function snapFile = saveSnapshot(rgbFrame, ids, locs, detFams)
    %SAVESNAPSHOT Save the current frame with detections burned in.

    logDir = fullfile(fileparts(mfilename('fullpath')), 'logs');
    if ~exist(logDir, 'dir'); mkdir(logDir); end
    timeStr = datestr(datetime('now'), 'yyyy-mm-dd_HH-MM-SS');
    snapFile = fullfile(logDir, sprintf('apriltag_minsize_snap_%s.png', timeStr));

    img = rgbFrame;
    for i = 1:length(ids)
        img = insertShape(img, 'Polygon', reshape(locs(:,:,i)', 1, []), ...
                          'Color', 'green', 'LineWidth', 4);
        img = insertText(img, mean(locs(:,:,i), 1), ...
                         sprintf('%s ID %d', famShort(detFams(i)), ids(i)), ...
                         'FontSize', 24, 'BoxColor', 'green');
    end
    imwrite(img, snapFile);
end
