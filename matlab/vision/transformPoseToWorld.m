function [worldPos, worldRot] = transformPoseToWorld(camPos, camRot, config)
    %TRANSFORMPOSETOWORLD Camera-frame AprilTag pose -> world/testbed frame.
    %
    % Single source of truth for the camera->world transform. Every caller
    % (getAprilTagPose, aprilTagOdometry_3D, aprilTagDistanceTest) should use
    % this instead of local copies of transformToWorld.
    %
    % UWB-repo port (2026-07-02) of MMS vision core/transformPoseToWorld.m —
    % logic unchanged.
    %
    % Two modes:
    %   1. REGISTERED (preferred): if config.extrinsics.available is true, a
    %      rigid transform (R, t) solved from AprilTag detections at surveyed
    %      testbed points (see registerWorldFrame.m) maps camera-frame
    %      coordinates into the surveyed world frame (the UWB anchor frame).
    %      This accounts for camera tilt, mount height error, and any
    %      offset/rotation between image axes and the testbed axes.
    %   2. LEGACY NADIR fallback: assumes a perfectly plumb, level camera at
    %      config.camera.height with world origin on the floor directly below
    %      it (X-right, Y-forward, Z-up).
    %
    % Inputs:
    %   camPos - 1x3 or 3x1 tag position in camera frame (meters), i.e.
    %            poses(i).Translation from readAprilTag
    %   camRot - 3x3 tag rotation in camera frame, i.e. poses(i).R
    %   config - struct from visionSystemConfig()
    %
    % Outputs:
    %   worldPos - 1x3 position in world frame (meters)
    %   worldRot - 3x3 tag orientation in world frame; yaw via
    %              rotm2eul(worldRot, 'XYZ')
    %
    % Note on the rotation convention: the tag body axes are re-expressed with
    % the same axis flip as the legacy code (R_flip = diag([1,-1,-1])), so a
    % tag lying flat on the floor reads roll ~= 0, pitch ~= 0 in both modes,
    % and yaw stays continuous when switching modes.

    R_flip = [1, 0, 0; 0, -1, 0; 0, 0, -1];  % legacy camera<->world axis flip

    if isfield(config, 'extrinsics') && config.extrinsics.available
        R = config.extrinsics.R;
        t = config.extrinsics.t(:);
        worldPos = (R * camPos(:) + t)';
        worldRot = R * camRot * R_flip';
    else
        worldPos = [camPos(1), -camPos(2), config.camera.height - camPos(3)];
        worldRot = R_flip * camRot * R_flip';
    end
end
