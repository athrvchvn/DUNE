function [position, orientation, tagID, isDetected] = getAprilTagPose(kinectObj, visionConfig, targetTagID)
    %GETAPRILTAGPOSE Get AprilTag position and orientation from Kinect frame
    %
    % This function captures a single frame from Kinect, detects AprilTags,
    % and returns the position and orientation in world coordinates.
    %
    % Inputs:
    %   kinectObj    - Kinect VideoDevice object (already initialized)
    %   visionConfig - Vision system configuration struct from visionSystemConfig()
    %   targetTagID  - (Optional) Specific tag ID to track (default: 0)
    %
    % Outputs:
    %   position     - [x, y, z] position in world frame (meters)
    %   orientation  - [roll, pitch, yaw] Euler angles in radians
    %   tagID        - ID of detected tag (or -1 if none detected)
    %   isDetected   - Boolean indicating if tag was detected
    %
    % Usage:
    %   config = visionSystemConfig();
    %   kinect = imaq.VideoDevice('kinect', 1, 'BGR_1920x1080');
    %   kinect.ReturnedColorSpace = 'rgb';
    %   [pos, orient, id, detected] = getAprilTagPose(kinect, config, 0);
    %
    % Based on: aprilTagOdometry_3D.m
    % Author: MMS Vision Team
    % Date: November 2025

    % Default outputs (no detection)
    position = [0, 0, 0];
    orientation = [0, 0, 0];
    tagID = -1;
    isDetected = false;

    % Default to base tag if not specified
    if nargin < 3
        targetTagID = visionConfig.apriltag.base.id;
    end

    % Select correct tag size based on target ID
    if targetTagID == 0
        tag_size = visionConfig.apriltag.base.size;  % 17.78cm (7 inches)
    elseif targetTagID == 2
        tag_size = visionConfig.apriltag.endEffector.size;  % 11cm
    else
        % Default to base tag size for unknown IDs
        tag_size = visionConfig.apriltag.base.size;
    end

    try
        % Capture frame from Kinect
        rgbFrame = step(kinectObj);
        rgbFrame = rgbFrame(:,:,[3,2,1]);  % BGR to RGB
        grayFrame = rgb2gray(rgbFrame);
        grayFrame = fliplr(grayFrame);     % Horizontal flip for Kinect

        % Detect AprilTags with correct size
        [ids, ~, poses] = readAprilTag(grayFrame, ...
                                       visionConfig.apriltag.base.family, ...
                                       visionConfig.camera.intrinsics, ...
                                       tag_size);

        % Check if any tags detected
        if isempty(ids)
            return;  % No tags detected
        end

        % Find target tag or use first detected tag
        tagIdx = find(ids == targetTagID, 1);
        if isempty(tagIdx)
            tagIdx = 1;  % Use first detected tag if target not found
        end

        % Extract tag ID
        tagID = ids(tagIdx);

        % Transform pose to world coordinates (registered extrinsics if
        % available, legacy nadir assumption otherwise)
        [position, worldRot] = transformPoseToWorld(poses(tagIdx).Translation, ...
                                                    poses(tagIdx).R, visionConfig);

        % Extract Euler angles [roll, pitch, yaw]
        orientation = rotm2eul(worldRot, 'XYZ');

        % Mark as detected
        isDetected = true;

    catch ME
        % Error during detection - return no detection
        warning('getAprilTagPose:detectionFailed', 'Detection failed - %s', ME.message);
        isDetected = false;
    end
end

% Coordinate transforms live in core/transformPoseToWorld.m (single source).
