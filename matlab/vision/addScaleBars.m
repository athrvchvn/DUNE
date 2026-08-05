function addScaleBars()
%ADDSCALEBARS Stand-alone scale-bar step for the overhead-camera calibration.
%
%   Decoupled from calibrateOverheadCamera's Stage B so you can anchor /
%   re-anchor the floor scale WITHOUT re-running the AprilTag sweep. Loads
%   the saved stage-B homography, lets you click tape-measured marks, reports
%   per-bar scale error, and (optionally) rescales the homography + updates
%   the effective tag size, then saves back into camera_calibration.mat.
%
%   Robust to the window being closed mid-run: it just reopens a fresh frame.
%
%   Run auditFloorScale afterwards to confirm every zone lands 0.99-1.01.

    config = visionSystemConfig();
    cc = config.camcal;
    if ~cc.available || isempty(cc.H_floor2norm)
        error(['No stage-B homography found in camera_calibration.mat. ' ...
               'Run calibrateOverheadCamera Stage B first.']);
    end
    pp = config.camera.principalPoint;
    nf = cc.norm_f;
    H  = cc.H_floor2norm;
    S0 = config.apriltag.base.sizeNominal;

    kinectObj = imaq.VideoDevice(config.camera.deviceName, ...
                                 config.camera.deviceID, ...
                                 config.camera.colorFormat);
    kinectObj.ReturnedColorSpace = config.camera.returnedColorSpace;
    cleanupObj = onCleanup(@() release(kinectObj)); %#ok<NASGU>

    figS = openFrame(kinectObj);
    bars = [];   % [trueD, modelD, trueD/modelD]
    while true
        a = input('Add a scale bar (two tape-measured marks)? (ENTER = yes, s = done): ', 's');
        if strcmpi(strtrim(a), 's'), break; end
        if ~ishghandle(figS), figS = openFrame(kinectObj); end   % reopen if closed
        figure(figS);
        fprintf('Click the TWO marks.\n');
        [ub, vb] = ginput(2);
        if numel(ub) < 2, fprintf('Need two clicks — bar discarded.\n'); continue; end
        plot(ub, vb, 'g+-', 'MarkerSize', 12, 'LineWidth', 2);
        D = str2double(input('True distance between them (m): ', 's'));
        if ~isfinite(D) || D <= 0, fprintf(2, 'Bad value, bar discarded.\n'); continue; end
        und = undistortPts([ub, vb], cc);
        pf  = homApply(inv(H), (und - pp) / nf); %#ok<MINV>
        d   = norm(diff(pf, 1, 1));
        bars(end + 1, :) = [D, d, D / d]; %#ok<AGROW>
        fprintf('  model says %.4f m vs true %.4f m -> scale error %+.2f%%\n', ...
                d, D, (d / D - 1) * 100);
    end
    if ishghandle(figS), close(figS); end
    if isempty(bars)
        fprintf('No bars entered — nothing changed.\n');
        return;
    end

    s = mean(bars(:, 3));
    errs = (bars(:, 2) ./ bars(:, 1) - 1) * 100;
    fprintf('\n%d bars — per-bar scale error:', size(bars, 1));
    fprintf(' %+.2f%%', errs);
    fprintf('\nMean scale error %+.2f%%  ->  correction factor %.4f\n', mean(errs), s);

    a = input(sprintf(['Apply the %.4f scale correction to the homography and save? ' ...
                       '(ENTER = yes, s = skip): '], s), 's');
    if strcmpi(strtrim(a), 's')
        fprintf('Not saved — homography unchanged.\n');
        return;
    end
    out = load(cc.file);
    out.H_floor2norm = H * diag([1 / s, 1 / s, 1]);
    out.scale_bars   = bars;
    out.tag_size_used = S0 * s;
    out.date = datestr(now, 'yyyy-mm-dd HH:MM:SS'); %#ok<TNOW1,DATST>
    save(cc.file, '-struct', 'out');
    fprintf(['Saved. Effective tag size now %.4f m (printed %.4f m).\n' ...
             'NEXT: run auditFloorScale — target every zone 0.99-1.01.\n'], S0 * s, S0);
end

%% ------------------------------------------------------------------------
function figS = openFrame(kinectObj)
    frame = grabFrame(kinectObj);
    figS = figure('Name', 'Scale bars: click two tape-measured marks', ...
                  'Position', [50, 50, 1500, 850]);
    imshow(frame); hold on;
end

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
