function aprilTagOdometry_3D()
    %APRILTAGODOMETRY_3D Track AprilTag movement and log the trajectory.
    %
    % UWB-repo port (2026-07-02) of MMS vision analysis/aprilTagOdometry_3D.m.
    % This is the ground-truth logger for the GT calibration campaign
    % (docs/gt_calibration_campaign.md): it tracks the AprilTag on the
    % dual-tag unit, plots live, and on Ctrl+C writes one CSV per tag ID with
    % PosixTime_s wall-clock timestamps for alignment with the UWB JSONL logs
    % (python/analysis/gt_align.py).
    %
    % Changes vs the MMS original: shared config/transform come from this
    % folder; CSVs are written to matlab/vision/logs/ (not the cwd); the
    % visualization floor/limits use the UWB testbed bounds.
    %
    %   Usage:
    %     aprilTagOdometry_3D()
    %
    %   Press Ctrl+C to stop execution and save data

    %% Configuration (single source of truth: visionSystemConfig.m)
    config = visionSystemConfig();
    intrinsics = config.camera.intrinsics;
    tagFamily = config.apriltag.base.family;
    tagSize = config.apriltag.base.size;
    kinectPos = config.camera.position;

    %% Initialize Kinect v2
    kinectObj = imaq.VideoDevice(config.camera.deviceName, ...
                                 config.camera.deviceID, ...
                                 config.camera.colorFormat);
    kinectObj.ReturnedColorSpace = config.camera.returnedColorSpace;

    %% Odometry Data Structures
    odometryData = struct();
    maxHistoryLength = 1000;  % initial capacity; arrays grow automatically

    % Initialize data storage for each potential tag ID (assuming max 50 tags)
    for tagID = 1:50
        odometryData.(sprintf('tag_%d', tagID)) = initializeTagData(maxHistoryLength);
    end

    %% Visualization Setup
    figure('Position', [50, 50, 1800, 900], 'Name', 'AprilTag Odometry Tracking System');

    %% Timing and Statistics
    startTime = tic;
    frameCount = 0;
    lastUpdateTime = 0;

    fprintf('AprilTag Odometry System Started (UWB GT logger)\n');
    fprintf('Press Ctrl+C to stop and save data\n');
    fprintf('Testbed: %.1fm x %.1fm, UWB anchor frame (origin = anchor 1)\n', ...
            config.workspace.xRange(2) - 0.5, config.workspace.yRange(2) - 0.5);
    fprintf('Kinect position: (%.1f, %.1f, %.1f) meters\n', kinectPos);
    if config.extrinsics.available
        fprintf('World frame: REGISTERED extrinsics (%s, RMSE %.1f mm)\n\n', ...
                config.extrinsics.file, config.extrinsics.rmse_m * 1000);
    else
        fprintf(['WARNING: world frame is the nadir assumption (no registration ' ...
                 'found). GT data logged like this CANNOT be aligned with the ' ...
                 'anchor frame - run registerWorldFrame.m first.\n\n']);
    end

    %% Main Tracking Loop
    try
        while true
            currentTime = toc(startTime);
            posixNow = posixtime(datetime('now'));  % absolute wall-clock (s), for UWB log alignment

            % Get and process frame
            rgbFrame = step(kinectObj);
            rgbFrame = rgbFrame(:,:,[3,2,1]);  % BGR to RGB
            grayFrame = rgb2gray(rgbFrame);
            grayFrame = fliplr(grayFrame);     % Horizontal flip

            % Detect AprilTags
            [ids, locs, poses] = readAprilTag(grayFrame, tagFamily, intrinsics, tagSize);

            % Update odometry data
            odometryData = updateOdometryData(odometryData, ids, poses, config, currentTime, posixNow);

            % Visualize every 0.1 seconds to maintain performance
            if currentTime - lastUpdateTime > 0.1
                visualizeOdometry(grayFrame, ids, locs, poses, kinectPos, odometryData, currentTime, config);
                lastUpdateTime = currentTime;
                frameCount = frameCount + 1;

                % Display frame rate
                if mod(frameCount, 10) == 0
                    fps = frameCount / currentTime;
                    fprintf('FPS: %.1f | Active Tags: %d | Time: %.1fs\n', fps, length(ids), currentTime);
                end
            end

            pause(0.01);  % Small delay
        end

    catch ME
        if ~strcmp(ME.identifier, 'MATLAB:interruption')
            fprintf('Error: %s\n', ME.message);
        end
    end

    %% Cleanup and Save Data
    fprintf('\nStopping tracking and saving data...\n');
    saveOdometryData(odometryData, startTime);
    generateTrajectoryReport(odometryData);

    release(kinectObj);
    fprintf('Odometry tracking completed successfully!\n');
end

function tagData = initializeTagData(maxLength)
    %INITIALIZETAGDATA Initialize data structure for a single tag
    tagData = struct();
    tagData.timestamps = zeros(maxLength, 1);
    tagData.posixTimes = zeros(maxLength, 1);     % absolute wall-clock (posixtime)
    tagData.positions = zeros(maxLength, 3);      % [x, y, z]
    tagData.orientations = zeros(maxLength, 3);   % Euler angles [roll, pitch, yaw]
    tagData.quaternions = zeros(maxLength, 4);    % [w, x, y, z]
    tagData.velocities = zeros(maxLength, 3);     % [vx, vy, vz]
    tagData.angularVel = zeros(maxLength, 3);     % [wx, wy, wz]
    tagData.accelerations = zeros(maxLength, 3);  % [ax, ay, az]
    tagData.distances = zeros(maxLength, 1);      % Distance from origin
    tagData.count = 0;                            % Number of valid entries
    tagData.lastSeen = 0;                         % Last detection time
    tagData.isActive = false;                     % Currently being tracked
end

function odometryData = updateOdometryData(odometryData, ids, poses, config, currentTime, posixNow)
    %UPDATEODOMETRYDATA Update trajectory data for all detected tags

    % Mark all tags as inactive initially
    tagFields = fieldnames(odometryData);
    for i = 1:length(tagFields)
        if isfield(odometryData.(tagFields{i}), 'isActive')
            odometryData.(tagFields{i}).isActive = false;
        end
    end

    % Process each detected tag
    for i = 1:length(ids)
        tagID = ids(i);
        fieldName = sprintf('tag_%d', tagID);

        if ~isfield(odometryData, fieldName)
            odometryData.(fieldName) = initializeTagData(1000);
        end

        tagData = odometryData.(fieldName);

        % Transform position and orientation (registered extrinsics if available)
        [worldPos, worldRot] = transformPoseToWorld(poses(i).Translation, poses(i).R, config);
        euler = rotm2eul(worldRot, 'XYZ');  % [roll, pitch, yaw]
        quat = rotm2quat(worldRot);         % [w, x, y, z]

        % Update data arrays
        newIndex = tagData.count + 1;
        if newIndex > length(tagData.timestamps)
            % Expand arrays if needed
            tagData = expandTagDataArrays(tagData);
        end

        tagData.timestamps(newIndex) = currentTime;
        tagData.posixTimes(newIndex) = posixNow;
        tagData.positions(newIndex, :) = worldPos;
        tagData.orientations(newIndex, :) = euler;
        tagData.quaternions(newIndex, :) = quat;
        tagData.distances(newIndex) = norm(worldPos);

        % Calculate velocities and accelerations
        if newIndex > 1
            dt = currentTime - tagData.timestamps(newIndex-1);
            if dt > 0
                % Linear velocity
                vel = (worldPos - tagData.positions(newIndex-1, :)) / dt;
                tagData.velocities(newIndex, :) = vel;

                % Angular velocity (simplified)
                dEuler = euler - tagData.orientations(newIndex-1, :);
                % Handle angle wrapping
                dEuler = wrapToPi(dEuler);
                tagData.angularVel(newIndex, :) = dEuler / dt;

                % Linear acceleration
                if newIndex > 2
                    prevVel = tagData.velocities(newIndex-1, :);
                    accel = (vel - prevVel) / dt;
                    tagData.accelerations(newIndex, :) = accel;
                end
            end
        end

        tagData.count = newIndex;
        tagData.lastSeen = currentTime;
        tagData.isActive = true;

        odometryData.(fieldName) = tagData;
    end
end

function tagData = expandTagDataArrays(tagData)
    %EXPANDTAGDATAARRAYS Expand arrays when they get full
    currentSize = length(tagData.timestamps);
    newSize = currentSize * 2;

    % Expand all arrays
    tagData.timestamps(newSize) = 0;
    tagData.posixTimes(newSize) = 0;
    tagData.positions(newSize, 3) = 0;
    tagData.orientations(newSize, 3) = 0;
    tagData.quaternions(newSize, 4) = 0;
    tagData.velocities(newSize, 3) = 0;
    tagData.angularVel(newSize, 3) = 0;
    tagData.accelerations(newSize, 3) = 0;
    tagData.distances(newSize) = 0;
end

function visualizeOdometry(rgbFrame, ids, locs, poses, kinectPos, odometryData, currentTime, config)
    %VISUALIZEODOMETRY Create comprehensive visualization of odometry data

    % Create 2x2 subplot layout

    %% Subplot 1: Camera View
    subplot(2, 2, 1);
    imshow(rgbFrame);
    title(sprintf('Camera View - Time: %.1fs', currentTime), 'FontSize', 12);
    hold on;

    % Draw detected tags
    if ~isempty(ids)
        for i = 1:length(ids)
            corners = locs(:,:,i);
            plot([corners(:,1); corners(1,1)], [corners(:,2); corners(1,2)], 'g-', 'LineWidth', 3);

            centerX = mean(corners(:,1));
            centerY = mean(corners(:,2));
            plot(centerX, centerY, 'yo', 'MarkerSize', 8, 'MarkerFaceColor', 'yellow');

            text(centerX, centerY-30, sprintf('ID: %d', ids(i)), ...
                'Color', 'yellow', 'FontSize', 14, 'FontWeight', 'bold', ...
                'HorizontalAlignment', 'center', 'BackgroundColor', 'black');
        end
    end
    hold off;

    %% Subplot 2: 3D Trajectory View
    subplot(2, 2, 2);
    plot3DTrajectory(odometryData, kinectPos, currentTime, config);

    %% Subplot 3: Position vs Time
    subplot(2, 2, 3);
    plotPositionHistory(odometryData, currentTime);

    %% Subplot 4: Orientation vs Time
    subplot(2, 2, 4);
    plotOrientationHistory(odometryData, currentTime);

    drawnow;
end

function plot3DTrajectory(odometryData, kinectPos, currentTime, config)
    %PLOT3DTRAJECTORY Plot 3D trajectories of all active tags
    cla;
    hold on;

    % Draw testbed floor (UWB anchor frame bounds)
    xr = config.workspace.xRange;
    yr = config.workspace.yRange;
    floorX = [xr(1), xr(2), xr(2), xr(1), xr(1)];
    floorY = [yr(1), yr(1), yr(2), yr(2), yr(1)];
    floorZ = [0, 0, 0, 0, 0];
    plot3(floorX, floorY, floorZ, 'k-', 'LineWidth', 2);
    patch(floorX(1:4), floorY(1:4), floorZ(1:4), [0.9 0.9 0.9], 'FaceAlpha', 0.3);

    % World coordinate axes (origin = anchor 1)
    axisLength = 1.0;
    plot3([0, axisLength], [0, 0], [0, 0], 'r-', 'LineWidth', 3);
    plot3([0, 0], [0, axisLength], [0, 0], 'g-', 'LineWidth', 3);
    plot3([0, 0], [0, 0], [0, axisLength], 'b-', 'LineWidth', 3);

    % Kinect position
    plot3(kinectPos(1), kinectPos(2), kinectPos(3), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'red');

    % Plot trajectories for all active tags
    colors = lines(10);  % Get different colors for different tags
    colorIndex = 1;

    tagFields = fieldnames(odometryData);
    activeTags = [];

    for i = 1:length(tagFields)
        tagData = odometryData.(tagFields{i});

        if tagData.count > 0 && (currentTime - tagData.lastSeen) < 2.0  % Show if seen in last 2 seconds
            tagID = str2double(tagFields{i}(5:end));  % Extract tag ID from field name
            activeTags = [activeTags, tagID];

            % Get valid trajectory points
            validIdx = 1:tagData.count;
            positions = tagData.positions(validIdx, :);

            if size(positions, 1) > 1
                % Plot trajectory line
                plot3(positions(:,1), positions(:,2), positions(:,3), ...
                      'Color', colors(colorIndex,:), 'LineWidth', 2, 'DisplayName', sprintf('Tag %d', tagID));

                % Plot start point (green)
                plot3(positions(1,1), positions(1,2), positions(1,3), ...
                      'go', 'MarkerSize', 8, 'MarkerFaceColor', 'green');

                % Plot current position (large colored marker)
                if tagData.isActive
                    plot3(positions(end,1), positions(end,2), positions(end,3), ...
                          'o', 'Color', colors(colorIndex,:), 'MarkerSize', 12, 'MarkerFaceColor', colors(colorIndex,:));

                    % Add velocity vector
                    if tagData.count > 1
                        vel = tagData.velocities(tagData.count, :) * 0.5;  % Scale for visibility
                        quiver3(positions(end,1), positions(end,2), positions(end,3), ...
                               vel(1), vel(2), vel(3), 0, 'Color', colors(colorIndex,:), 'LineWidth', 2);
                    end
                end
            else
                % Single point
                plot3(positions(1,1), positions(1,2), positions(1,3), ...
                      'o', 'Color', colors(colorIndex,:), 'MarkerSize', 12, 'MarkerFaceColor', colors(colorIndex,:));
            end

            colorIndex = colorIndex + 1;
            if colorIndex > size(colors, 1)
                colorIndex = 1;
            end
        end
    end

    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title(sprintf('3D Trajectories (Active Tags: %s)', mat2str(activeTags)));
    grid on; axis equal;
    xlim([xr(1) - 0.5, xr(2) + 0.5]);
    ylim([yr(1) - 0.5, yr(2) + 0.5]);
    zlim([0, 4]);
    view(-37.5, 15);
    if ~isempty(activeTags)
        legend('show', 'Location', 'best');
    end
    hold off;
end

function plotPositionHistory(odometryData, currentTime)
    %PLOTPOSITIONHISTORY Plot X, Y, Z positions over time
    cla;
    hold on;

    colors = lines(10);
    colorIndex = 1;
    legendEntries = {};

    tagFields = fieldnames(odometryData);
    for i = 1:length(tagFields)
        tagData = odometryData.(tagFields{i});

        if tagData.count > 5  % Only plot if we have enough data points
            tagID = str2double(tagFields{i}(5:end));
            validIdx = 1:tagData.count;

            times = tagData.timestamps(validIdx);
            positions = tagData.positions(validIdx, :);

            % Plot X, Y, Z with different line styles
            plot(times, positions(:,1), '-', 'Color', colors(colorIndex,:), 'LineWidth', 1.5);
            plot(times, positions(:,2), '--', 'Color', colors(colorIndex,:), 'LineWidth', 1.5);
            plot(times, positions(:,3), ':', 'Color', colors(colorIndex,:), 'LineWidth', 2);

            legendEntries{end+1} = sprintf('Tag %d X', tagID);
            legendEntries{end+1} = sprintf('Tag %d Y', tagID);
            legendEntries{end+1} = sprintf('Tag %d Z', tagID);

            colorIndex = colorIndex + 1;
            if colorIndex > size(colors, 1)
                colorIndex = 1;
            end
        end
    end

    xlabel('Time (s)');
    ylabel('Position (m)');
    title('Position vs Time');
    grid on;
    if ~isempty(legendEntries)
        legend(legendEntries, 'Location', 'best');
    end
    hold off;
end

function plotOrientationHistory(odometryData, currentTime)
    %PLOTORIENTATIONHISTORY Plot Euler angles over time
    cla;
    hold on;

    colors = lines(10);
    colorIndex = 1;
    legendEntries = {};

    tagFields = fieldnames(odometryData);
    for i = 1:length(tagFields)
        tagData = odometryData.(tagFields{i});

        if tagData.count > 5
            tagID = str2double(tagFields{i}(5:end));
            validIdx = 1:tagData.count;

            times = tagData.timestamps(validIdx);
            orientations = rad2deg(tagData.orientations(validIdx, :));  % Convert to degrees

            % Plot Roll, Pitch, Yaw
            plot(times, orientations(:,1), '-', 'Color', colors(colorIndex,:), 'LineWidth', 1.5);
            plot(times, orientations(:,2), '--', 'Color', colors(colorIndex,:), 'LineWidth', 1.5);
            plot(times, orientations(:,3), ':', 'Color', colors(colorIndex,:), 'LineWidth', 2);

            legendEntries{end+1} = sprintf('Tag %d Roll', tagID);
            legendEntries{end+1} = sprintf('Tag %d Pitch', tagID);
            legendEntries{end+1} = sprintf('Tag %d Yaw', tagID);

            colorIndex = colorIndex + 1;
            if colorIndex > size(colors, 1)
                colorIndex = 1;
            end
        end
    end

    xlabel('Time (s)');
    ylabel('Orientation (degrees)');
    title('Orientation (Roll/Pitch/Yaw) vs Time');
    grid on;
    if ~isempty(legendEntries)
        legend(legendEntries, 'Location', 'best');
    end
    hold off;
end

function saveOdometryData(odometryData, startTime)
    %SAVEODOMETRYDATA Save all tracking data to CSV files in matlab/vision/logs/

    logDir = fullfile(fileparts(mfilename('fullpath')), 'logs');
    if ~exist(logDir, 'dir')
        mkdir(logDir);
    end

    % Create timestamp for filename
    timeStr = datestr(now, 'yyyy-mm-dd_HH-MM-SS');

    tagFields = fieldnames(odometryData);
    for i = 1:length(tagFields)
        tagData = odometryData.(tagFields{i});

        if tagData.count > 0
            tagID = str2double(tagFields{i}(5:end));
            filename = fullfile(logDir, sprintf('apriltag_%d_trajectory_%s.csv', tagID, timeStr));

            % Prepare data for export
            validIdx = 1:tagData.count;
            exportData = [
                tagData.timestamps(validIdx), ...           % Time (relative)
                tagData.posixTimes(validIdx), ...           % Absolute wall-clock
                tagData.positions(validIdx, :), ...         % X, Y, Z
                rad2deg(tagData.orientations(validIdx, :)), ... % Roll, Pitch, Yaw (degrees)
                tagData.quaternions(validIdx, :), ...       % Quaternions
                tagData.velocities(validIdx, :), ...        % Velocities
                rad2deg(tagData.angularVel(validIdx, :)), ...   % Angular velocities (degrees/s)
                tagData.accelerations(validIdx, :), ...     % Accelerations
                tagData.distances(validIdx)                 % Distance from origin
            ];

            % Column headers
            headers = {'Time_s', 'PosixTime_s', 'X_m', 'Y_m', 'Z_m', 'Roll_deg', 'Pitch_deg', 'Yaw_deg', ...
                      'Qw', 'Qx', 'Qy', 'Qz', 'Vx_ms', 'Vy_ms', 'Vz_ms', ...
                      'Wx_degs', 'Wy_degs', 'Wz_degs', 'Ax_ms2', 'Ay_ms2', 'Az_ms2', 'Dist_m'};

            % Write to CSV
            writetable(array2table(exportData, 'VariableNames', headers), filename);
            fprintf('Saved trajectory data for Tag %d: %s (%d points)\n', tagID, filename, tagData.count);
        end
    end
end

function generateTrajectoryReport(odometryData)
    %GENERATETRAJECTORYREPORT Generate statistical report of all trajectories

    fprintf('\n=== TRAJECTORY ANALYSIS REPORT ===\n');

    tagFields = fieldnames(odometryData);
    for i = 1:length(tagFields)
        tagData = odometryData.(tagFields{i});

        if tagData.count > 5  % Only analyze tags with sufficient data
            tagID = str2double(tagFields{i}(5:end));
            validIdx = 1:tagData.count;

            positions = tagData.positions(validIdx, :);
            velocities = tagData.velocities(validIdx, :);

            % Calculate statistics
            totalDistance = sum(sqrt(sum(diff(positions).^2, 2)));
            avgVelocity = mean(sqrt(sum(velocities.^2, 2)));
            maxVelocity = max(sqrt(sum(velocities.^2, 2)));

            posRange = max(positions) - min(positions);
            duration = tagData.timestamps(tagData.count) - tagData.timestamps(1);

            fprintf('\nTag %d Statistics:\n', tagID);
            fprintf('  Duration: %.2f seconds\n', duration);
            fprintf('  Data points: %d\n', tagData.count);
            fprintf('  Total distance: %.2f m\n', totalDistance);
            fprintf('  Average velocity: %.2f m/s\n', avgVelocity);
            fprintf('  Maximum velocity: %.2f m/s\n', maxVelocity);
            fprintf('  Position range: X=%.2fm, Y=%.2fm, Z=%.2fm\n', posRange(1), posRange(2), posRange(3));
            fprintf('  Final position: (%.2f, %.2f, %.2f) m\n', positions(end,1), positions(end,2), positions(end,3));
        end
    end
    fprintf('\n=== END REPORT ===\n');
end

% Coordinate transforms live in transformPoseToWorld.m (single source).
