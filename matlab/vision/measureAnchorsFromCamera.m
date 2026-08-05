function W = measureAnchorsFromCamera(nAnchors)
%MEASUREANCHORSFROMCAMERA Survey the anchor layout with the calibrated Kinect.
%
%   Click each anchor's ANTENNA in ID order. Each click is undistorted, mapped
%   through the stage-B floor homography, then CORRECTED for the antenna height
%   (z ~ 0.22 m) — a raw floor-plane map would sit a few % too far from nadir
%   (parallax), up to ~10 cm at the edges. The de-parallaxed points are the true
%   (x,y). The world frame is then defined (anchor 1 = origin, +X toward anchor
%   2, the rest on +Y) and written to matlab/config/anchors.json.
%
%   Prints the baseline (a1-a2) and main diagonal (a1-a3) so you can cross-check
%   the CAMERA's metric against a tape (independent sanity check).
%
%   Needs the stage-B homography + calibrated height (calibrateOverheadCamera).

    if nargin < 1, nAnchors = 5; end
    config = visionSystemConfig();
    cc = config.camcal;
    if ~cc.available || isempty(cc.H_floor2norm)
        error('Need the stage-B homography (run calibrateOverheadCamera Stage B first).');
    end
    pp = config.camera.principalPoint;
    nf = cc.norm_f;
    H  = cc.H_floor2norm;
    h  = config.camera.height;          % calibrated mount height (~4.066 m)
    anchorZ = 0.24;                     % antenna height above floor (m)
    kInward = (h - anchorZ) / h;        % parallax de-projection factor

    % Nadir on the floor (centre of the radial de-parallax)
    if isfield(cc, 'nadir_px') && numel(cc.nadir_px) == 2 && all(isfinite(cc.nadir_px))
        nadirPx = cc.nadir_px;
    else
        nadirPx = pp;
    end
    nadirFloor = homApply(inv(H), (undistortPts(nadirPx, cc) - pp) / nf); %#ok<MINV>

    kinectObj = imaq.VideoDevice(config.camera.deviceName, ...
                                 config.camera.deviceID, ...
                                 config.camera.colorFormat);
    kinectObj.ReturnedColorSpace = config.camera.returnedColorSpace;
    cleanupObj = onCleanup(@() release(kinectObj)); %#ok<NASGU>

    frame = grabFrame(kinectObj);
    fig = figure('Name', 'Measure anchors from camera — click each ANTENNA in ID order', ...
                 'Position', [50, 50, 1500, 850]);
    P = nan(nAnchors, 2);   % true floor (x,y) per anchor
    while true
        clf(fig); imshow(frame); hold on;
        title('Click each anchor ANTENNA in ID order 1..N (zoom with scroll first)', 'FontSize', 12);
        for k = 1:nAnchors
            xlabel(sprintf('>>> Click anchor %d ANTENNA   [%d/%d]', k, k, nAnchors), ...
                   'FontSize', 13, 'FontWeight', 'bold', 'Color', 'red');
            [u, v] = ginput(1);
            pf = homApply(inv(H), (undistortPts([u, v], cc) - pp) / nf); %#ok<MINV>
            P(k, :) = nadirFloor + (pf - nadirFloor) * kInward;   % de-parallax to z=0.22
            plot(u, v, 'g+', 'MarkerSize', 14, 'LineWidth', 2);
            text(u + 12, v, sprintf('a%d', k), 'Color', 'green', 'FontWeight', 'bold');
        end
        xlabel('All anchors clicked.', 'Color', 'black');
        if ~strcmpi(strtrim(input('Accept clicks? (ENTER = yes, r = redo): ', 's')), 'r')
            break;
        end
    end
    if ishghandle(fig), close(fig); end

    %% Define the world frame: a1 = origin, +X toward a2, a3 side = +Y
    o  = P(1, :);
    ex = P(2, :) - o; ex = ex / norm(ex);
    ey = [-ex(2), ex(1)];
    if dot(P(3, :) - o, ey) < 0, ey = -ey; end     % put the a3/a4 row on +Y
    W = (P - o) * [ex.', ey.'];                     % N x 2 world coords

    %% Report + tape cross-check
    fprintf('\n=== Anchor layout measured from the camera ===\n');
    fprintf('%-8s %-10s %-10s\n', 'anchor', 'x (m)', 'y (m)');
    for k = 1:nAnchors
        fprintf('a%-7d %-10.3f %-10.3f\n', k, W(k, 1), W(k, 2));
    end
    baseline = norm(P(2, :) - P(1, :));
    if nAnchors >= 3, diag13 = norm(P(3, :) - P(1, :)); else, diag13 = NaN; end
    fprintf(['\nCAMERA metric vs your tape:\n  a1-a2 (baseline) = %.3f m  (tape ~3.00)\n' ...
             '  a1-a3 (diagonal) = %.3f m  (tape ~4.11)\n'], baseline, diag13);
    fprintf('If these match the tape to ~1-2 cm, the camera survey is trustworthy.\n');

    %% Write anchors.json
    if strcmpi(strtrim(input('\nWrite this to anchors.json? (ENTER = yes, s = skip): ', 's')), 's')
        fprintf('Not written.\n'); return;
    end
    out = struct();
    out.dim = 2;
    mrg = 0.5;
    out.bounds = [min(W(:,1))-mrg, max(W(:,1))+mrg, ...
                  min(W(:,2))-mrg, max(W(:,2))+mrg, 0.0, 3.0];
    anchors = struct('id', {}, 'x', {}, 'y', {}, 'z', {});
    for k = 1:nAnchors
        anchors(k) = struct('id', k, 'x', round(W(k,1),3), 'y', round(W(k,2),3), 'z', anchorZ);
    end
    out.anchors = anchors;
    out.layout = 'camera_survey_2026-07-19';
    jsonFile = fullfile(fileparts(mfilename('fullpath')), '..', 'config', 'anchors.json');
    fid = fopen(jsonFile, 'w');
    fwrite(fid, jsonencode(out, 'PrettyPrint', true));
    fclose(fid);
    fprintf('Wrote %s\nNEXT: run registerWorldFrameClicks (its RMSE should now be small).\n', jsonFile);
end

%% ------------------------------------------------------------------------
function frame = grabFrame(kinectObj)
    frame = step(kinectObj);
    frame = frame(:, :, [3, 2, 1]);
    frame = fliplr(frame);
end

function q = homApply(H, pts)
    n = size(pts, 1);
    w = (H * [pts, ones(n, 1)]')';
    q = w(:, 1:2) ./ w(:, 3);
end
