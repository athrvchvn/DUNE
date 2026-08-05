function results = auditFloorScale()
    %AUDITFLOORSCALE Quick floor-scale accuracy audit ("is 1 m really 1 m?").
    %
    % Click pairs of tape-verified floor marks and compare the true
    % separation against what the current camera model predicts. Run it with
    % ~6 pairs: 1 m along x at image CENTRE, near the LEFT edge, near the
    % RIGHT edge, and the same along y (add diagonals if curious). The
    % centre-vs-edge / x-vs-y pattern identifies the error source:
    %
    %   centre right, edges short  -> radial lens distortion
    %                                 (run calibrateOverheadCamera stage A)
    %   uniformly wrong everywhere -> f/h product wrong or camera tilt
    %                                 (run stages B-C)
    %
    % Marks must lie ON THE FLOOR plane (z = 0 relative to sand surface).
    % Reports both models when an in-situ calibration exists:
    %   NADIR: (pixel - pp)/f * h with the current config constants
    %   CALIB: undistort + stage-B floor homography (config.camcal)
    %
    % Output: struct array with per-pair fields (trueD, dNadir, dCalib,
    % angleDeg, meanRadiusPx, zone).

    config = visionSystemConfig();
    pp = config.camera.principalPoint;
    f  = config.camera.focalLength(1);
    h  = config.camera.height;
    cc = config.camcal;
    haveCal = cc.available && ~isempty(cc.H_floor2norm);
    if ~haveCal
        fprintf('(no in-situ calibration loaded — reporting the nadir model only)\n');
    end

    kinectObj = imaq.VideoDevice(config.camera.deviceName, ...
                                 config.camera.deviceID, ...
                                 config.camera.colorFormat);
    kinectObj.ReturnedColorSpace = config.camera.returnedColorSpace;
    cleanupObj = onCleanup(@() release(kinectObj)); %#ok<NASGU>
    frame = step(kinectObj);
    frame = frame(:, :, [3, 2, 1]);
    frame = fliplr(frame);

    fig = figure('Name', 'Floor-scale audit: click mark pairs', ...
                 'Position', [50, 50, 1500, 850]); %#ok<NASGU>
    imshow(frame); hold on;
    title('Click pairs of tape-verified marks (centre + edges, x and y)', 'FontSize', 12);

    results = struct('trueD', {}, 'dNadir', {}, 'dCalib', {}, ...
                     'angleDeg', {}, 'meanRadiusPx', {}, 'zone', {});
    fprintf('\n%-4s %-8s %-10s %-10s %-10s %-8s %-7s\n', ...
            '#', 'true(m)', 'nadir(m)', 'calib(m)', 'angle', 'radius', 'zone');
    while true
        ans_ = input('Measure a pair? (ENTER = yes, q = quit): ', 's');
        if strcmpi(strtrim(ans_), 'q'), break; end
        fprintf('Click the TWO marks.\n');
        [u, v] = ginput(2);
        if numel(u) < 2, continue; end
        Ds = input('True separation in metres (ENTER = 1.0): ', 's');
        D = str2double(Ds);
        if isempty(strtrim(Ds)), D = 1.0; end
        if ~isfinite(D) || D <= 0, fprintf(2, 'Bad value, discarded.\n'); continue; end

        % nadir model (current config constants, no distortion)
        pn = ([u, v] - pp) / f * h;
        dN = norm(diff(pn, 1, 1));
        % calibrated model
        dC = NaN;
        if haveCal
            und = undistortPts([u, v], cc);
            pf = homApply(inv(cc.H_floor2norm), (und - pp) / cc.norm_f); %#ok<MINV>
            dC = norm(diff(pf, 1, 1));
        end

        seg = diff([u, v], 1, 1);
        ang = mod(atan2d(seg(2), seg(1)), 180);
        radius = mean(sqrt(sum(([u, v] - pp).^2, 2)));
        if radius < 350, zone = 'centre';
        elseif radius < 650, zone = 'mid';
        else, zone = 'edge';
        end

        k = numel(results) + 1;
        results(k) = struct('trueD', D, 'dNadir', dN, 'dCalib', dC, ...
                            'angleDeg', ang, 'meanRadiusPx', radius, 'zone', zone);
        plot(u, v, 'g+-', 'MarkerSize', 12, 'LineWidth', 2);
        text(mean(u), mean(v) - 14, sprintf('%d', k), 'Color', 'green', ...
             'FontWeight', 'bold', 'FontSize', 11);
        fprintf('%-4d %-8.3f %-10.3f %-10.3f %-8.0f %-8.0f %-7s\n', ...
                k, D, dN, dC, ang, radius, zone);
    end
    if isempty(results), fprintf('No pairs measured.\n'); return; end

    %% Summary: scale ratio (measured / true) by zone and direction
    fprintf('\n=== Scale ratio measured/true (1.000 = perfect) ===\n');
    fprintf('%-18s %-14s %-14s %-4s\n', 'group', 'nadir', 'calib', 'n');
    zones = {'centre', 'mid', 'edge'};
    for zi = 1:numel(zones)
        sel = strcmp({results.zone}, zones{zi});
        printGroup(zones{zi}, results(sel));
    end
    angs = [results.angleDeg];
    printGroup('x-ish (<30deg)',  results(angs < 30 | angs > 150));
    printGroup('y-ish (60-120)',  results(angs >= 60 & angs <= 120));
    printGroup('diagonal',        results((angs >= 30 & angs < 60) | (angs > 120 & angs <= 150)));
    fprintf(['\nReading it: edges-only shrink -> lens distortion. Uniform error ' ...
             '-> f/h or tilt.\nx-vs-y difference at the CENTRE -> tilt (or fx~=fy, ' ...
             'unlikely).\n']);
end

function printGroup(name, rr)
    if isempty(rr), return; end
    rN = mean([rr.dNadir] ./ [rr.trueD]);
    rC = mean([rr.dCalib] ./ [rr.trueD]);
    fprintf('%-18s %-14.4f %-14.4f %-4d\n', name, rN, rC, numel(rr));
end

function q = homApply(H, pts)
    n = size(pts, 1);
    w = (H * [pts, ones(n, 1)]')';
    q = w(:, 1:2) ./ w(:, 3);
end
