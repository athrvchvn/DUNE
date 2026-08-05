function aprilTagDistanceTest()
    %APRILTAGDISTANCETEST Test distance measurement accuracy with 1 m moves.
    %
    % UWB-repo port (2026-07-02) of MMS vision examples/aprilTagDistanceTest.m.
    % Intrinsics/height now come from visionSystemConfig() (the MMS original
    % hardcoded a divergent height of 4.0 m) and positions go through the
    % shared transformPoseToWorld(). Test locations are expressed in the UWB
    % anchor frame for the 6.3 x 3.1 m testbed.
    %
    % This is run-book Step 0.2: validate the Kinect's relative accuracy
    % across the floor, INCLUDING the edges, before trusting it as ground
    % truth. Expect < 3 cm error on 1 m movements.
    %
    %   Test Protocol:
    %   - Test at 4 locations spread across the testbed
    %   - At each location, move the tag unit exactly 1 m along a tape measure
    %   - Compare against the camera-measured displacement
    %
    %   Usage:
    %     aprilTagDistanceTest()
    %
    %   Controls:
    %     'r' = Record reference position
    %     'm' = Record moved position (after 1m movement)
    %     'n' = Skip to next test
    %     'q' = Quit and save results

    %% Initialize
    fprintf('==============================================\n');
    fprintf('AprilTag Distance Measurement Test (UWB testbed)\n');
    fprintf('==============================================\n\n');

    %% Configuration (single source of truth: visionSystemConfig.m)
    config = visionSystemConfig();
    intrinsics = config.camera.intrinsics;
    tagFamily = config.apriltag.base.family;
    tagSize = config.apriltag.base.size;
    kinectPos = config.camera.position;
    paramLabel = 'ORIGINAL';

    fprintf('Camera parameters (%s):\n', paramLabel);
    fprintf('  fx = %.2f, fy = %.2f\n', config.camera.focalLength(1), config.camera.focalLength(2));
    fprintf('  cx = %.2f, cy = %.2f\n', config.camera.principalPoint(1), config.camera.principalPoint(2));
    fprintf('  height = %.2f m\n\n', config.camera.height);

    %% Initialize Kinect
    fprintf('Initializing Kinect v2...\n');
    kinectObj = imaq.VideoDevice(config.camera.deviceName, ...
                                 config.camera.deviceID, ...
                                 config.camera.colorFormat);
    kinectObj.ReturnedColorSpace = config.camera.returnedColorSpace;
    fprintf('[OK] Kinect initialized\n\n');

    %% Define Distance Tests (locations in the UWB anchor frame, m)
    % Testbed is ~6.3 x 3.1 m, origin at anchor 1, +X toward anchor 2.
    % 1 m movements stay well inside the bed from these starting points.
    testSequence = {
        % Location 1: Center
        {'Center +X', [3.15, 1.55], '+X (toward anchor 2)', 'Place unit at CENTER (~3.15, 1.55), then move +1m in +X'};
        {'Center +Y', [3.15, 1.55], '+Y (toward anchor 4 side)', 'Place unit at CENTER, then move +1m in +Y'};

        % Location 2: East side (near anchor 2/3 edge)
        {'East -X', [5.3, 1.55], '-X (back toward center)', 'Place unit at EAST side (~5.3, 1.55), then move 1m in -X'};
        {'East +Y', [5.3, 0.8], '+Y', 'Place unit at EAST-SOUTH (~5.3, 0.8), then move +1m in +Y'};

        % Location 3: West side (near anchor 1/4 edge)
        {'West +X', [1.0, 1.55], '+X', 'Place unit at WEST side (~1.0, 1.55), then move +1m in +X'};
        {'West +Y', [1.0, 0.8], '+Y', 'Place unit at WEST-SOUTH (~1.0, 0.8), then move +1m in +Y'};

        % Location 4: North edge (anchor 5 side)
        {'North -Y', [3.15, 2.5], '-Y (back toward center)', 'Place unit at NORTH edge (~3.15, 2.5), then move 1m in -Y'};
        {'South +X', [2.0, 0.6], '+X', 'Place unit at SOUTH edge (~2.0, 0.6), then move +1m in +X'};
    };

    expectedDistance = 1.0;  % meters

    %% Test Protocol
    fprintf('TEST PROTOCOL:\n');
    fprintf('---------------------------------------------\n');
    fprintf('For each test:\n');
    fprintf('  1. Place the tag unit at the starting position\n');
    fprintf('  2. Press ''r'' to record REFERENCE position\n');
    fprintf('  3. Move the unit EXACTLY 1 meter (use a tape measure!)\n');
    fprintf('  4. Press ''m'' to record MOVED position\n');
    fprintf('  5. System calculates measured distance\n\n');
    fprintf('Total tests: %d\n', size(testSequence, 1));
    fprintf('---------------------------------------------\n\n');

    %% Data Storage
    results = struct();
    results.timestamp = datetime('now');
    results.cameraParams = paramLabel;
    results.focalLength = config.camera.focalLength;
    results.principalPoint = config.camera.principalPoint;
    results.expectedDistance = expectedDistance;
    results.measurements = [];

    %% Create Figure
    fig = figure('Position', [50, 50, 1600, 900], ...
                 'Name', 'AprilTag Distance Test', ...
                 'KeyPressFcn', @(src, evt) keyPressCallback(evt));

    userCommand = '';
    currentTestIdx = 1;
    testState = 'waiting_ref';  % 'waiting_ref' or 'waiting_moved'
    refPosition = [];
    instructionDisplayed = false;

    %% Main Loop
    try
        fprintf('Starting distance test...\n\n');

        while currentTestIdx <= size(testSequence, 1)
            % Display instruction
            if ~instructionDisplayed
                fprintf('\n========================================\n');
                fprintf('TEST %d/%d: %s\n', currentTestIdx, size(testSequence, 1), ...
                       testSequence{currentTestIdx}{1});
                fprintf('========================================\n');
                fprintf('Direction: %s\n', testSequence{currentTestIdx}{3});
                fprintf('%s\n', testSequence{currentTestIdx}{4});

                if strcmp(testState, 'waiting_ref')
                    fprintf('>> Press ''r'' to record REFERENCE position\n');
                else
                    fprintf('>> Move unit EXACTLY 1m, then press ''m'' to record MOVED position\n');
                end
                instructionDisplayed = true;
            end

            % Capture frame
            rgbFrame = step(kinectObj);
            rgbFrame = rgbFrame(:,:,[3,2,1]);
            rgbFrame = fliplr(rgbFrame);
            grayFrame = rgb2gray(rgbFrame);

            % Detect AprilTag
            [ids, locs, poses] = readAprilTag(grayFrame, tagFamily, intrinsics, tagSize);

            % Visualize
            visualizeDistance(fig, rgbFrame, ids, locs, poses, config, ...
                            testSequence, currentTestIdx, testState, refPosition, ...
                            expectedDistance, results);

            % Handle user input
            if ~isempty(userCommand)
                if strcmp(userCommand, 'r') && strcmp(testState, 'waiting_ref')
                    % Record reference position
                    if ~isempty(ids)
                        refPosition = transformPoseToWorld(poses(1).Translation, poses(1).R, config);
                        fprintf('\n[REFERENCE] Position recorded: (%.3f, %.3f, %.3f) m\n', refPosition);
                        fprintf('>> Now move unit EXACTLY 1m in %s direction\n', testSequence{currentTestIdx}{3});
                        testState = 'waiting_moved';
                        instructionDisplayed = false;
                    else
                        fprintf('[ERROR] No AprilTag detected! Try again.\n');
                    end

                elseif strcmp(userCommand, 'm') && strcmp(testState, 'waiting_moved')
                    % Record moved position
                    if ~isempty(ids)
                        movedPosition = transformPoseToWorld(poses(1).Translation, poses(1).R, config);

                        % Calculate distance
                        displacement = movedPosition - refPosition;
                        measuredDistance = norm(displacement);
                        distanceError = measuredDistance - expectedDistance;
                        errorPercent = (distanceError / expectedDistance) * 100;

                        measurement = struct();
                        measurement.testIdx = currentTestIdx;
                        measurement.testName = testSequence{currentTestIdx}{1};
                        measurement.startLocation = testSequence{currentTestIdx}{2};
                        measurement.direction = testSequence{currentTestIdx}{3};
                        measurement.refPosition = refPosition;
                        measurement.movedPosition = movedPosition;
                        measurement.displacement = displacement;
                        measurement.expectedDistance = expectedDistance;
                        measurement.measuredDistance = measuredDistance;
                        measurement.distanceError = distanceError;
                        measurement.errorPercent = errorPercent;

                        results.measurements = [results.measurements; measurement];

                        fprintf('\n[RECORDED] Test %d: %s\n', currentTestIdx, testSequence{currentTestIdx}{1});
                        fprintf('  Reference:  (%.3f, %.3f, %.3f) m\n', refPosition);
                        fprintf('  Moved:      (%.3f, %.3f, %.3f) m\n', movedPosition);
                        fprintf('  Displacement: (%.3f, %.3f, %.3f) m\n', displacement);
                        fprintf('  Expected distance: %.3f m\n', expectedDistance);
                        fprintf('  Measured distance: %.3f m\n', measuredDistance);
                        fprintf('  Error:       %.3f m (%.1f mm) [%.1f%%]\n', ...
                               distanceError, distanceError*1000, errorPercent);

                        % Move to next test
                        currentTestIdx = currentTestIdx + 1;
                        testState = 'waiting_ref';
                        refPosition = [];
                        instructionDisplayed = false;
                    else
                        fprintf('[ERROR] No AprilTag detected! Try again.\n');
                    end

                elseif strcmp(userCommand, 'n')
                    fprintf('[SKIPPED] Test %d\n', currentTestIdx);
                    currentTestIdx = currentTestIdx + 1;
                    testState = 'waiting_ref';
                    refPosition = [];
                    instructionDisplayed = false;

                elseif strcmp(userCommand, 'q')
                    fprintf('\n[QUIT] Stopping test...\n');
                    break;
                end

                userCommand = '';
            end

            pause(0.05);
        end

    catch ME
        if ~strcmp(ME.identifier, 'MATLAB:interruption')
            fprintf('\n[ERROR] %s\n', ME.message);
        end
    end

    %% Generate Report
    fprintf('\n\n==============================================\n');
    fprintf('DISTANCE TEST COMPLETE\n');
    fprintf('==============================================\n');

    if ~isempty(results.measurements)
        generateDistanceReport(results);
        saveDistanceResults(results);
    else
        fprintf('No measurements recorded.\n');
    end

    %% Cleanup
    release(kinectObj);
    fprintf('\n[OK] Test completed!\n');

    function keyPressCallback(evt)
        userCommand = evt.Key;
    end
end

function visualizeDistance(fig, rgbFrame, ids, locs, poses, config, ...
                          testSequence, currentTestIdx, testState, refPosition, ...
                          expectedDistance, results)
    %VISUALIZEDISTANCE Display camera view with distance measurement info

    % Check if figure is still valid, recreate if needed
    if ~ishandle(fig) || ~isvalid(fig)
        warning('Figure was closed. Cannot continue visualization.');
        return;
    end

    figure(fig);

    %% Left: Camera View
    subplot(1, 2, 1);
    imshow(rgbFrame);
    hold on;

    % Draw detected AprilTag
    if ~isempty(ids)
        for i = 1:length(ids)
            corners = locs(:,:,i);
            % Draw tag outline (green if detected)
            plot([corners(:,1); corners(1,1)], ...
                 [corners(:,2); corners(1,2)], ...
                 'g-', 'LineWidth', 4);
        end
    end

    % Status indicator
    if ~isempty(ids)
        rectangle('Position', [20, 20, 180, 40], 'FaceColor', [0, 0.5, 0, 0.8], 'EdgeColor', 'white', 'LineWidth', 2);
        text(110, 40, 'TAG OK', 'Color', 'white', 'FontSize', 14, ...
            'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    else
        rectangle('Position', [20, 20, 180, 40], 'FaceColor', [0.7, 0, 0, 0.8], 'EdgeColor', 'white', 'LineWidth', 2);
        text(110, 40, 'NO TAG', 'Color', 'white', 'FontSize', 14, ...
            'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    end

    % Show state
    if strcmp(testState, 'waiting_ref')
        stateText = 'WAITING FOR REFERENCE';
        stateColor = [0.2, 0.4, 0.8];
    else
        stateText = 'WAITING FOR MOVED POSITION';
        stateColor = [0.8, 0.4, 0.2];
    end
    rectangle('Position', [20, 80, 400, 40], 'FaceColor', [stateColor, 0.8], 'EdgeColor', 'white', 'LineWidth', 2);
    text(220, 100, stateText, 'Color', 'white', 'FontSize', 12, ...
        'FontWeight', 'bold', 'HorizontalAlignment', 'center');

    hold off;

    % Title
    title(sprintf('Test %d/%d: %s | Direction: %s', ...
                 currentTestIdx, size(testSequence,1), ...
                 testSequence{currentTestIdx}{1}, ...
                 testSequence{currentTestIdx}{3}), ...
          'FontSize', 12, 'FontWeight', 'bold');

    %% Right: Info Panel
    subplot(1, 2, 2);
    cla;
    axis off;

    % Display current test info
    yPos = 0.95;
    text(0.05, yPos, sprintf('TEST %d/%d: %s', currentTestIdx, size(testSequence,1), ...
                            testSequence{currentTestIdx}{1}), ...
        'FontSize', 13, 'FontWeight', 'bold', 'Color', 'blue');
    yPos = yPos - 0.08;

    text(0.05, yPos, sprintf('Start Location: (%.1f, %.1f)', ...
                            testSequence{currentTestIdx}{2}), ...
        'FontSize', 11, 'Color', [0.3, 0.3, 0.3]);
    yPos = yPos - 0.06;

    text(0.05, yPos, sprintf('Direction: %s', testSequence{currentTestIdx}{3}), ...
        'FontSize', 11, 'FontWeight', 'bold', 'Color', [0.5, 0, 0.5]);
    yPos = yPos - 0.06;

    text(0.05, yPos, sprintf('Expected distance: %.2f m', expectedDistance), ...
        'FontSize', 11, 'Color', [0.2, 0.2, 0.2]);
    yPos = yPos - 0.10;

    % Show reference position if recorded
    if ~isempty(refPosition)
        text(0.05, yPos, 'REFERENCE Position:', ...
            'FontSize', 11, 'FontWeight', 'bold', 'Color', [0, 0.5, 0]);
        yPos = yPos - 0.06;
        text(0.1, yPos, sprintf('X = %.3f m', refPosition(1)), 'FontSize', 10, 'Color', [0, 0.5, 0]);
        yPos = yPos - 0.05;
        text(0.1, yPos, sprintf('Y = %.3f m', refPosition(2)), 'FontSize', 10, 'Color', [0, 0.5, 0]);
        yPos = yPos - 0.05;
        text(0.1, yPos, sprintf('Z = %.3f m', refPosition(3)), 'FontSize', 10, 'Color', [0, 0.5, 0]);
        yPos = yPos - 0.10;
    end

    % Show current position if tag is detected
    if ~isempty(ids)
        worldPos = transformPoseToWorld(poses(1).Translation, poses(1).R, config);

        text(0.05, yPos, 'CURRENT Position:', ...
            'FontSize', 11, 'FontWeight', 'bold', 'Color', [0, 0, 0.7]);
        yPos = yPos - 0.06;
        text(0.1, yPos, sprintf('X = %.3f m', worldPos(1)), 'FontSize', 10, 'Color', [0, 0, 0.7]);
        yPos = yPos - 0.05;
        text(0.1, yPos, sprintf('Y = %.3f m', worldPos(2)), 'FontSize', 10, 'Color', [0, 0, 0.7]);
        yPos = yPos - 0.05;
        text(0.1, yPos, sprintf('Z = %.3f m', worldPos(3)), 'FontSize', 10, 'Color', [0, 0, 0.7]);
        yPos = yPos - 0.10;

        % If reference exists, show distance
        if ~isempty(refPosition)
            displacement = worldPos - refPosition;
            currentDistance = norm(displacement);

            text(0.05, yPos, 'Distance from Reference:', ...
                'FontSize', 11, 'FontWeight', 'bold', 'Color', 'red');
            yPos = yPos - 0.06;
            text(0.1, yPos, sprintf('%.3f m (%.0f mm)', currentDistance, currentDistance*1000), ...
                'FontSize', 11, 'Color', 'red', 'FontWeight', 'bold');
            yPos = yPos - 0.08;

            % Show displacement vector
            text(0.05, yPos, sprintf('Displacement: (%.3f, %.3f, %.3f) m', displacement), ...
                'FontSize', 9, 'Color', [0.5, 0.5, 0.5]);
        end
    end

    yPos = 0.30;
    text(0.05, yPos, 'Controls:', 'FontSize', 11, 'FontWeight', 'bold');
    yPos = yPos - 0.06;
    text(0.1, yPos, '''r'' = Record reference position', 'FontSize', 10);
    yPos = yPos - 0.05;
    text(0.1, yPos, '''m'' = Record moved position', 'FontSize', 10);
    yPos = yPos - 0.05;
    text(0.1, yPos, '''n'' = Skip test', 'FontSize', 10);
    yPos = yPos - 0.05;
    text(0.1, yPos, '''q'' = Quit and save', 'FontSize', 10);

    % Progress
    yPos = 0.10;
    numCompleted = length(results.measurements);
    text(0.05, yPos, sprintf('Progress: %d/%d tests completed', ...
                            numCompleted, size(testSequence,1)), ...
        'FontSize', 10, 'FontWeight', 'bold');

    xlim([0, 1]);
    ylim([0, 1]);

    drawnow;
end

function generateDistanceReport(results)
    %GENERATEDISTANCEREPORT Generate report of distance test

    fprintf('\n========================================\n');
    fprintf('DISTANCE MEASUREMENT RESULTS\n');
    fprintf('========================================\n');
    fprintf('Camera Parameters: %s\n', results.cameraParams);
    fprintf('  fx = %.2f, fy = %.2f\n', results.focalLength(1), results.focalLength(2));
    fprintf('  cx = %.2f, cy = %.2f\n', results.principalPoint(1), results.principalPoint(2));
    fprintf('Expected distance: %.3f m\n', results.expectedDistance);
    fprintf('Total measurements: %d\n\n', length(results.measurements));

    if ~isempty(results.measurements)
        distanceErrors = [];
        errorPercents = [];

        fprintf('Individual Measurements:\n');
        fprintf('----------------------------------------\n');
        for i = 1:length(results.measurements)
            m = results.measurements(i);
            fprintf('%d. %s [%s]\n', m.testIdx, m.testName, m.direction);
            fprintf('   Expected:  %.3f m\n', m.expectedDistance);
            fprintf('   Measured:  %.3f m\n', m.measuredDistance);
            fprintf('   Error:     %.3f m (%.1f mm) [%.1f%%]\n', ...
                   m.distanceError, m.distanceError*1000, m.errorPercent);
            fprintf('   Displacement: (%.3f, %.3f, %.3f) m\n\n', m.displacement);

            distanceErrors = [distanceErrors; abs(m.distanceError)];
            errorPercents = [errorPercents; abs(m.errorPercent)];
        end

        fprintf('========================================\n');
        fprintf('DISTANCE ACCURACY STATISTICS\n');
        fprintf('========================================\n');
        fprintf('Mean absolute error:   %.3f m (%.1f mm) [%.1f%%]\n', ...
               mean(distanceErrors), mean(distanceErrors)*1000, mean(errorPercents));
        fprintf('Std dev:               %.3f m (%.1f mm) [%.1f%%]\n', ...
               std(distanceErrors), std(distanceErrors)*1000, std(errorPercents));
        fprintf('Max error:             %.3f m (%.1f mm) [%.1f%%]\n', ...
               max(distanceErrors), max(distanceErrors)*1000, max(errorPercents));
        fprintf('Min error:             %.3f m (%.1f mm) [%.1f%%]\n', ...
               min(distanceErrors), min(distanceErrors)*1000, min(errorPercents));
        fprintf('RMS error:             %.3f m (%.1f mm)\n', ...
               rms(distanceErrors), rms(distanceErrors)*1000);
        fprintf('========================================\n');
    end
end

function saveDistanceResults(results)
    %SAVEDISTANCERESULTS Save results to matlab/vision/logs/

    logDir = fullfile(fileparts(mfilename('fullpath')), 'logs');
    if ~exist(logDir, 'dir')
        mkdir(logDir);
    end

    timeStr = datestr(results.timestamp, 'yyyy-mm-dd_HH-MM-SS');
    paramStr = lower(results.cameraParams);

    % Save MAT file
    matFilename = fullfile(logDir, sprintf('apriltag_distance_%s_%s.mat', paramStr, timeStr));
    save(matFilename, 'results');
    fprintf('\n[SAVED] MAT file: %s\n', matFilename);

    % Save CSV file
    if ~isempty(results.measurements)
        csvFilename = fullfile(logDir, sprintf('apriltag_distance_%s_%s.csv', paramStr, timeStr));

        data = [];
        headers = {'Test', 'TestName', 'Direction', 'Start_X', 'Start_Y', ...
                  'Ref_X_m', 'Ref_Y_m', 'Ref_Z_m', ...
                  'Moved_X_m', 'Moved_Y_m', 'Moved_Z_m', ...
                  'Disp_X_m', 'Disp_Y_m', 'Disp_Z_m', ...
                  'Expected_Dist_m', 'Measured_Dist_m', 'Error_m', 'Error_mm', 'Error_Percent'};

        for i = 1:length(results.measurements)
            m = results.measurements(i);
            row = [m.testIdx, string(m.testName), string(m.direction), ...
                  m.startLocation, m.refPosition, m.movedPosition, ...
                  m.displacement, m.expectedDistance, m.measuredDistance, ...
                  m.distanceError, m.distanceError*1000, m.errorPercent];
            data = [data; row];
        end

        T = array2table(data, 'VariableNames', headers);
        writetable(T, csvFilename);
        fprintf('[SAVED] CSV file: %s\n', csvFilename);
    end
end

% Coordinate transforms live in transformPoseToWorld.m (single source).
