function [position, orientation, tagID, isDetected] = getAprilTagPoseFromFrame(rgbFrame, visionConfig, targetTagID)
    %GETAPRILTAGPOSEFROMFRAME AprilTag world pose from an ALREADY-GRABBED frame.
    %
    % Frame-in sibling of getAprilTagPose(): identical detection + calibrated
    % camera->world transform, but takes an RGB frame instead of a Kinect handle.
    % This lets a caller that already owns the camera (e.g. the rover teleop loop,
    % which grabs frames through the MMS HardwareManipulatorControl) apply the
    % DUNE camera calibration + registered world extrinsics to its own frames,
    % without opening a second handle on the single-owner Kinect device.
    %
    % The frame must be RGB in the SAME convention hw.getColorFrame() returns
    % (step() then BGR->RGB), i.e. NOT yet greyscaled or flipped — this function
    % does the rgb2gray + fliplr that the DUNE world registration assumes.
    %
    % Inputs:
    %   rgbFrame     - HxWx3 uint8 RGB frame (from hw.getColorFrame()).
    %   visionConfig - struct from visionSystemConfig() (DUNE calibration).
    %   targetTagID  - tag id to track (default: config base id).
    %
    % Outputs match getAprilTagPose(): position [x y z] m (world/anchor frame),
    % orientation [roll pitch yaw] rad, tagID, isDetected.

    position = [0, 0, 0];
    orientation = [0, 0, 0];
    tagID = -1;
    isDetected = false;

    if nargin < 3
        targetTagID = visionConfig.apriltag.base.id;
    end

    % Tag size by id (same selection as getAprilTagPose).
    if targetTagID == 2
        tag_size = visionConfig.apriltag.endEffector.size;
    else
        tag_size = visionConfig.apriltag.base.size;
    end

    try
        grayFrame = rgb2gray(rgbFrame);
        grayFrame = fliplr(grayFrame);     % world registration is in this convention

        [ids, ~, poses] = readAprilTag(grayFrame, ...
                                       visionConfig.apriltag.base.family, ...
                                       visionConfig.camera.intrinsics, ...
                                       tag_size);
        if isempty(ids)
            return;
        end

        tagIdx = find(ids == targetTagID, 1);
        if isempty(tagIdx), tagIdx = 1; end
        tagID = ids(tagIdx);

        [position, worldRot] = transformPoseToWorld(poses(tagIdx).Translation, ...
                                                    poses(tagIdx).R, visionConfig);
        orientation = rotm2eul(worldRot, 'XYZ');
        isDetected = true;
    catch ME
        warning('getAprilTagPoseFromFrame:detectionFailed', 'Detection failed - %s', ME.message);
        isDetected = false;
    end
end
