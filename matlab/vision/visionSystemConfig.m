function config = visionSystemConfig()
    %VISIONSYSTEMCONFIG Kinect v2 ground-truth config for the UWB RTLS testbed.
    %
    %   UWB-repo port (2026-07-02) of the MMS vision repo's config, trimmed to
    %   what the ground-truth calibration campaign needs (docs/
    %   gt_calibration_campaign.md). Function names intentionally match the
    %   MMS originals so the run-book commands are unchanged — do NOT keep
    %   D:\MMS_Codebase\matlab\vision* on the MATLAB path at the same time.
    %
    %   Usage:
    %     config = visionSystemConfig();
    %     intrinsics = config.camera.intrinsics;
    %
    %   Coordinate frames:
    %     World  = UWB anchor frame (matlab/config/anchors.json): origin at
    %              anchor 1, +X toward anchor 2, Z up. Established by
    %              registerWorldFrame(); until then a plumb-camera (nadir)
    %              fallback is used with origin on the floor below the camera.
    %     Camera = X-right, Y-down, Z-forward/depth; mounted overhead looking
    %              down at the ground plane.

    config = struct();

    %% Camera (intrinsics validated 2025-10-29 on this Kinect; keep ORIGINAL
    %  parameters, no distortion correction — corrected sets performed worse)
    config.camera = struct();
    config.camera.model = 'Kinect v2';
    config.camera.imageSize = [1080, 1920];          % [height, width] px
    config.camera.focalLength = [1050.0, 1050.0];    % [fx, fy] px
    config.camera.principalPoint = [960.0, 540.0];   % [cx, cy] px
    config.camera.radialDistortion = [0, 0];
    config.camera.tangentialDistortion = [0, 0];
    config.camera.intrinsics = cameraIntrinsics(...
        config.camera.focalLength, ...
        config.camera.principalPoint, ...
        config.camera.imageSize);

    % Mount height: VERIFY against a tape measure before each campaign
    % (run-book Step 0). Only used by the nadir fallback and sanity checks —
    % the registered extrinsics supersede it.
    config.camera.height = 3.43;                     % m above floor
    config.camera.position = [0, 0, config.camera.height];
    config.camera.orientation = 'downward';

    % Device settings (Image Acquisition Toolbox, Kinect for Windows support)
    config.camera.deviceName = 'kinect';
    config.camera.deviceID = 1;
    config.camera.colorFormat = 'BGR_1920x1080';
    config.camera.returnedColorSpace = 'rgb';

    %% World-frame extrinsics (camera -> UWB anchor frame)
    % Written by registerWorldFrame() from AprilTag detections at surveyed
    % floor points. When present, transformPoseToWorld() uses this rigid
    % transform instead of the nadir assumption.
    config.extrinsics = struct('available', false, 'R', [], 't', [], ...
                               'rmse_m', NaN, 'file', '');
    regFile = fullfile(fileparts(mfilename('fullpath')), ...
                       'calibration_data', 'world_registration.mat');
    if exist(regFile, 'file')
        reg = load(regFile);
        config.extrinsics.available = true;
        config.extrinsics.R = reg.R_cam2world;
        config.extrinsics.t = reg.t_cam2world;
        config.extrinsics.rmse_m = reg.rmse_m;
        config.extrinsics.file = regFile;
        if isfield(reg, 'registeredDate')
            config.extrinsics.registeredDate = reg.registeredDate;
        end
    end

    %% In-situ camera calibration (calibrateOverheadCamera.m)
    % Distortion (k1/k2/centre), pixel<->floor homography, calibrated camera
    % height / nadir / focal. Applied via undistortPts()/distortPts() and by
    % overriding the nominal focalLength/height below. Valid only in the
    % fliplr'd pixel convention.
    config.camcal = struct('available', false, 'k1', 0, 'k2', 0, ...
                           'dist_center', config.camera.principalPoint, ...
                           'norm_f', config.camera.focalLength(1), ...
                           'H_floor2norm', [], 'h', NaN, ...
                           'nadir_px', [NaN, NaN], 'tilt_deg', NaN, ...
                           'f_est', NaN, 'file', '');
    ccFile = fullfile(fileparts(mfilename('fullpath')), ...
                      'calibration_data', 'camera_calibration.mat');
    if exist(ccFile, 'file')
        cc = load(ccFile);
        fn = fieldnames(cc);
        for k = 1:numel(fn), config.camcal.(fn{k}) = cc.(fn{k}); end
        config.camcal.available = true;
        config.camcal.file = ccFile;
        % Calibrated focal length / mount height supersede the nominal
        % constants (originals kept as *Nominal / heightTape for reference)
        config.camera.focalLengthNominal = config.camera.focalLength;
        config.camera.heightTape = config.camera.height;
        if isfield(cc, 'f_est') && isfinite(cc.f_est)
            config.camera.focalLength = [cc.f_est, cc.f_est];
            config.camera.intrinsics = cameraIntrinsics(...
                config.camera.focalLength, ...
                config.camera.principalPoint, ...
                config.camera.imageSize);
        end
        if isfield(cc, 'h') && isfinite(cc.h)
            config.camera.height = cc.h;
            config.camera.position = [0, 0, config.camera.height];
        end
    end

    %% AprilTag on the tag unit
    config.apriltag = struct();
    config.apriltag.base = struct();
    config.apriltag.base.family = 'tag36h11';
    % EFFECTIVE size, depth-calibrated 2026-07-02 (this codebase only):
    % nominal print is 17.78 cm, but with size=0.1778 vision placed the tag
    % plate at z=0.043 m where a ruler says 0.192 m -> depth reads 4.6% long.
    % 0.1778 / ((3.43-0.043)/(3.43-0.192)) = 0.1700. Scales all camera-frame
    % translations; the click registration (homography, anchored to taped
    % anchor coords) is independent of this and stays valid.
    config.apriltag.base.size = 0.1700;
    config.apriltag.base.sizeNominal = 0.1778;
    config.apriltag.base.id = 0;
    config.apriltag.base.mounting = 'rigid plate on dual-tag unit, mid-baseline, plate z=0.192 m';
    % Lever arms from the AprilTag CENTER to each UWB tag antenna, in the
    % AprilTag body frame [x y z] (m) — the --lever argument for
    % python/analysis/gt_align.py. From registerWorldFrameClicks 2026-07-02,
    % XY rescaled to the taped 0.480 m baseline (clicks gave 0.454);
    % dz = antenna 0.22 - plate 0.192. Valid WITH the effective size above.
    config.apriltag.base.lever_uwb_0xF0 = [-0.255, -0.007, 0.028];
    config.apriltag.base.lever_uwb_0xF1 = [ 0.225, -0.027, 0.028];

    %% Testbed (UWB anchor frame; matches matlab/config/anchors.json)
    config.workspace = struct();
    config.workspace.xRange = [-0.5, 6.8];   % m (anchors span 0..6.3)
    config.workspace.yRange = [-0.5, 3.6];   % m (anchors span 0..3.1)
    config.workspace.zRange = [0.0, 3.0];    % m

    %% Camera<->world axis flip used by the nadir fallback
    config.coordinates = struct();
    config.coordinates.transforms = struct();
    config.coordinates.transforms.cameraToWorld = [1,  0,  0;
                                                   0, -1,  0;
                                                   0,  0, -1];

    %% Summary when called with no output
    if nargout == 0
        fprintf('=== UWB GT vision config ===\n');
        fprintf('Camera: %s, %dx%d, fx=fy=%.0f, height %.2f m\n', ...
                config.camera.model, config.camera.imageSize(2), ...
                config.camera.imageSize(1), config.camera.focalLength(1), ...
                config.camera.height);
        fprintf('AprilTag: %s id %d, %.2f cm\n', config.apriltag.base.family, ...
                config.apriltag.base.id, config.apriltag.base.size * 100);
        if config.extrinsics.available
            fprintf('World frame: REGISTERED (RMSE %.1f mm, %s)\n', ...
                    config.extrinsics.rmse_m * 1000, config.extrinsics.file);
        else
            fprintf(['World frame: NADIR FALLBACK — run registerWorldFrame() ' ...
                     'to align with the UWB anchor frame\n']);
        end
        if config.camcal.available
            fprintf(['Camera calib: LOADED (k1 %+.4f, k2 %+.4f, f %.1f, ' ...
                     'h %.3f m, tilt %.2f deg)\n'], config.camcal.k1, ...
                    config.camcal.k2, config.camera.focalLength(1), ...
                    config.camera.height, config.camcal.tilt_deg);
        else
            fprintf(['Camera calib: NONE — nominal constants; run ' ...
                     'calibrateOverheadCamera() (see auditFloorScale() first)\n']);
        end
    end
end
