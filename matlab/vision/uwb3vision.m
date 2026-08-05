function uwb3vision()
%UWB3VISION Put the DUNE UWB overhead-camera toolchain first on the path.
%
%   Guarantees d:\UWB_3\matlab\vision wins over any duplicate copies of the
%   shared function names (e.g. D:\UWB_modules_new\matlab\vision) regardless
%   of the current folder, and prints the active camera calibration so you
%   can see at a glance what the tools will use. Called automatically by
%   startup.m; safe to re-run at any time (e.g. after you cd elsewhere).

    here = fileparts(mfilename('fullpath'));
    addpath(here);                        % addpath prepends -> takes precedence

    dup = which('auditFloorScale.m', '-all');
    if numel(dup) > 1
        fprintf(2, '[UWB] %d copies of the vision tools on the path — using the first:\n', ...
                numel(dup));
        fprintf('   (active)   %s\n', dup{1});
        for i = 2:numel(dup)
            fprintf(2, '   (shadowed) %s\n', dup{i});
        end
    end

    ccFile = fullfile(here, 'calibration_data', 'camera_calibration.mat');
    if exist(ccFile, 'file')
        cc = load(ccFile);
        h  = getf(cc, 'h', NaN);
        fcal = isfield(cc, 'f_est') && isfinite(cc.f_est);
        f  = 1050; if fcal, f = cc.f_est; end
        hasA = isfield(cc, 'k1') && cc.k1 ~= 0;
        hasB = isfield(cc, 'H_floor2norm') && ~isempty(cc.H_floor2norm);
        fprintf(['[UWB] vision ready: %s\n' ...
                 '      h=%.3f m, f=%.0f px%s | distortion %s | floor-homography %s\n'], ...
                here, h, f, tern(fcal, ' (calibrated)', ' (nominal)'), ...
                tern(hasA, 'ON', 'OFF'), tern(hasB, 'ON', 'OFF'));
    else
        fprintf('[UWB] vision ready: %s (no camera_calibration.mat yet)\n', here);
    end
end

function v = getf(s, name, dflt)
    if isfield(s, name), v = s.(name); else, v = dflt; end
end

function s = tern(c, a, b)
    if c, s = a; else, s = b; end
end
