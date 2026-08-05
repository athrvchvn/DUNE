% rover_teleop_uwb.m
% PS4-teleoperated rover run with SIMULTANEOUS UWB capture (DUNE step 7.4/7.5).
%
% PORTED from D:\MMS_Codebase\matlab\slip_analysis\
%              test_random_excitation_slip_measurement.m
% The MMS codebase is used READ-ONLY (addpath only). Nothing there is modified;
% every change for DUNE lives in THIS file.
%
% WHY: the rover already carries an AprilTag that the Kinect tracks, so driving
% it with the two UWB tags aboard gives, on one clock:
%     AprilTag pose  = GROUND TRUTH
%     UWB sweeps     = what our estimator sees
% That is the moving-truth dataset step 7.5 needs to score EKF vs MHE vs the
% rigid two-tag MHE.
%
% SAFETY NOTE: the MHE is deliberately NOT run inside the control loop. This
% loop drives a moving rover and services the emergency stop; an IPOPT solve can
% occasionally take ~90 ms and would delay E-stop. The loop only drains and logs
% UWB sweeps (microseconds). Run the estimator offline afterwards - the result is
% identical because the sweeps are all that it consumes.
%
% Controls (PS4) - unchanged from the original:
%   D-pad Up/Down:    V +/- (sticky)        D-pad Left/Right: omega +/-
%   R1: reverse       Circle: E-STOP        Triangle: reset    Square: omega=0
%   L2: print state   PS button: exit and save
% Keyboard backup on the status figure: SPACE / ESC / Q = emergency stop.
%
% Data (one .mat): commands, AprilTag truth, rover Teensy IMU, encoders,
% steering, PLUS data.uwb (per-sweep ranges/rx/fp per tag + tag-240 IMU tail).
% Both teleop samples and UWB sweeps carry POSIX timestamps, so alignment is
% exact rather than inferred.

clear; close all; clc;

%% ========================================
%% CONFIGURATION
%% ========================================

% --- MMS codebase (READ-ONLY dependency: HardwareManipulatorControl etc.) ---
MMS_ROOT = 'D:\MMS_Codebase\matlab';

% --- Hardware ---
COM_PORT = 'COM8';   % ← PER MACHINE. Windows 'COM8'; Linux '/dev/ttyUSB0';
                     %   macOS '/dev/tty.usbserial-*'. See docs/getting_started.md.
APRILTAG_ID = 0;

% --- Ground-truth source for the AprilTag pose ---
%   "dune" - grab the Kinect frame via hw.getColorFrame() and run the DUNE
%            calibrated vision pipeline (visionSystemConfig + registered world
%            extrinsics) on it. Truth lands DIRECTLY in the UWB anchor frame and
%            uses the in-situ camera calibration + world registration this
%            project maintains under matlab/vision/calibration_data.
%   "mms"  - legacy: hw.getRoverPose(), which uses the MMS RoverControl's own
%            hard-coded intrinsics/height and its analytic nadir transform.
% Both only affect the LOGGED truth (control is manual); "dune" is the source
% the anchor-geometry / delay calibration was built against, so it makes the
% rover truth and the UWB solve share one calibrated frame.
TRUTH_SOURCE = "dune";     % "dune" | "mms"

% --- UWB capture (DUNE addition) ---
UWB_ENABLE   = true;
% Transport must match how the tag firmware is built:
%   "udp"    - both tags run UWB_COORD_RING + UWB_HOSTLINK_UDP and broadcast
%              independently over WiFi (the long-proven setup).
%   "serial" - the tags run UWB_COORD_WIRE: the IMU master owns the schedule,
%              grants the other tag its slot over the UART cable, and forwards
%              both tags' lines down ONE USB link.
% Either way the ingest is tag-agnostic - dune.TagUdp and dune.TagSerial share
% start/drain/drainEvents/stop and the consumer demuxes on s.tag - so nothing
% downstream of this switch changes.
UWB_TRANSPORT = "serial";    % "udp" | "serial"
UWB_PORT     = 4100;      % HostLink UDP broadcast port ("udp" only)
UWB_TAG_COM  = "COM12";        % ← PER MACHINE. Master tag's port ("serial";
                               %   "" = auto-detect). Linux: "/dev/ttyUSB0".
UWB_FRONT_TAG = 241;      % no IMU, mounted in front
UWB_REAR_TAG  = 240;      % BNO085, mounted behind
UWB_BASELINE  = 0.527;    % m, antenna centre-to-centre (measured 52.7 cm)

% AprilTag centre relative to the UWB rig, in ROVER BODY axes (metres):
%   x = forward (rear tag -> front tag), y = left.
% MEASURED 2026-07-22 (user diagram + confirmation):
%   baseline 52.7 cm  =>  half = 26.35 cm; rear tag 240 at x = -0.2635
%   AprilTag centre is 15 cm forward of the REAR tag (240, the IMU one)
%       ->  x = -0.2635 + 0.15 = -0.1135  (11.35 cm BEHIND the rig centre)
%   AprilTag is 10 cm to the rover's RIGHT  ->  y = -0.10  (y is +left)
% Used in analysis as: truth_centre = apriltag_xy - R(yaw) * offset
% (the lever arm rotates with heading, so it must be de-rotated, not subtracted).
APRILTAG_OFFSET_BODY = [-0.1135, -0.10];

% Frame relation between the Kinect/AprilTag world and the UWB anchor world is
% NOT assumed. Both are logged RAW; fit the 2D rigid transform offline.
FRAME_NOTE = 'apriltag and uwb frames logged raw; transform fitted offline';

% --- PS4 control increments ---
V_INCREMENT = 0.01;        % m/s per button press
OMEGA_INCREMENT = 0.01;    % rad/s per button press
V_MAX = 0.10;              % m/s
OMEGA_MAX = 0.107;         % rad/s

% --- Timing ---
CONTROL_RATE = 20;         % Hz, PS4 input polling target
STATUS_PRINT_INTERVAL = 1;
DISPLAY_UPDATE_INTERVAL = 0.5;
MAX_RETRIES = 3;           % IMU / encoder read retries (cheap serial)
RETRY_DELAY = 0.03;

% --- Loop scheduling (responsiveness fix, 2026-07-22) ---
% The heavy work (AprilTag detect ~150-280 ms, five serial commands per
% velocity send) used to run EVERY iteration, dragging the loop to ~3 Hz -
% at which point a quick button tap falls entirely between two polls and is
% never seen. Now the PS4 is polled fast and the heavy work is scheduled:
SENSOR_PERIOD    = 0.35;   % s between sensor-log samples (~3 Hz, what the
                           %   hardware achieves anyway - unchanged data rate)
APRILTAG_RETRIES = 1;      % was 3: a missed detection cost 500-900 ms in
                           %   retries; offline analysis interpolates gaps fine
VEL_KEEPALIVE    = 1.0;    % s: velocity is sent ON CHANGE + this keepalive
                           %   (motors latch their last command - the original
                           %   already relied on that while idle)
UWB_DRAIN_PERIOD = 0.15;   % s between UDP drains (an EMPTY Java poll costs
                           %   ~15 ms; packets buffer in the OS meanwhile)

% --- Rover physical parameters ---
WHEEL_RADIUS = 0.07;
GEAR_RATIO = 241;
WHEELBASE = 0.6;
TRACK_WIDTH = 0.70;

%% ========================================
%% ADD PATHS (read-only deps + DUNE)
%% ========================================

DUNE_ROOT = fileparts(fileparts(mfilename('fullpath')));   % ...\UWB_3\matlab
addpath(DUNE_ROOT);                                        % dune.* package

mmsDirs = { fullfile(MMS_ROOT, 'hardware'), ...
            fullfile(MMS_ROOT, 'research', 'hierarchical_task_decomposition', 'hardware'), ...
            fullfile(MMS_ROOT, 'kinematics') };
for k = 1:numel(mmsDirs)
    if ~isfolder(mmsDirs{k})
        error('rover_teleop_uwb:missingDep', ...
              ['MMS dependency folder not found:\n  %s\n' ...
               'Set MMS_ROOT at the top of this file.'], mmsDirs{k});
    end
    addpath(mmsDirs{k});
end
addpath(fullfile(DUNE_ROOT, 'vision'));   % DUNE calibrated vision (must win over MMS)
fprintf('Paths: DUNE %s + %d MMS dependency folders (read-only)\n', ...
        DUNE_ROOT, numel(mmsDirs));

% Load the DUNE ground-truth vision calibration when selected. Its registered
% world extrinsics put the AprilTag straight into the UWB anchor frame.
visionCfg = [];
if lower(TRUTH_SOURCE) == "dune"
    visionCfg = visionSystemConfig();
    if ~visionCfg.extrinsics.available
        warning(['DUNE world registration not found (matlab/vision/calibration_data/' ...
                 'world_registration.mat) — truth will use the plumb-camera nadir ' ...
                 'fallback, NOT the anchor frame. Run registerWorldFrame first.']);
    else
        fprintf('Ground truth: DUNE vision, registered world frame (RMSE %.1f mm)\n', ...
                1000*visionCfg.extrinsics.rmse_m);
    end
else
    fprintf('Ground truth: MMS legacy hw.getRoverPose()\n');
end

%% ========================================
%% INITIALIZE HARDWARE
%% ========================================

fprintf('\n=== PS4 rover run with UWB capture ===\n\n');
fprintf('Initializing hardware...\n');

hw = HardwareManipulatorControl(COM_PORT, 'aprilTagID', APRILTAG_ID);
hw.setNoPivotMode(true);  % Ackermann-only

hw.initializeKinect();
pause(1);

fprintf('Verifying AprilTag detection (%s truth)...\n', TRUTH_SOURCE);
aprilTagOK = false;
for retry = 1:MAX_RETRIES
    [roverPos, ~, det] = getTruthPose(hw, TRUTH_SOURCE, visionCfg, APRILTAG_ID, 1, 0);
    if det
        fprintf('  AprilTag detected at [%.2f, %.2f, %.2f] m\n', roverPos);
        aprilTagOK = true;
        break;
    end
    pause(RETRY_DELAY);
end
if ~aprilTagOK
    error('Cannot detect AprilTag ID %d after %d retries', APRILTAG_ID, MAX_RETRIES);
end

fprintf('Verifying rover IMU...\n');
imuOK = false;
for retry = 1:MAX_RETRIES
    try
        imu_test = hw.getTeensyIMU();
        if ~isempty(imu_test) && isfield(imu_test, 'available') && imu_test.available
            fprintf('  IMU available (accuracy: %d/3)\n', imu_test.accuracy);
            imuOK = true;
            break;
        end
    catch
        pause(RETRY_DELAY);
    end
end
if ~imuOK, warning('Teensy IMU not available - data will be NaN'); end

fprintf('Verifying motor encoders...\n');
encoderOK = false;
for retry = 1:MAX_RETRIES
    try
        encoder_test = hw.getEncoderSpeeds();
        if ~isempty(encoder_test) && length(encoder_test) == 6
            fprintf('  Encoders responding: [%d, %d, %d, %d, %d, %d] motor RPM\n', ...
                    round(encoder_test));
            encoderOK = true;
            break;
        end
    catch
        pause(RETRY_DELAY);
    end
end
if ~encoderOK, warning('Encoder read failed - data may be incomplete'); end

fprintf('  Hardware initialization complete!\n\n');

%% ========================================
%% INITIALIZE UWB CAPTURE (DUNE addition)
%% ========================================

A = dune.loadAnchors();
uwbM = numel(A.ids);
tu = [];
if UWB_ENABLE
    switch lower(string(UWB_TRANSPORT))
        case "udp"
            fprintf('Starting UWB UDP capture on port %d ...\n', UWB_PORT);
            tu = dune.TagUdp(UWB_PORT);
        case "serial"
            if strlength(UWB_TAG_COM) == 0
                fprintf('Starting UWB serial capture (auto-detect port) ...\n');
                tu = dune.TagSerial();      % errors if the port is ambiguous
            else
                fprintf('Starting UWB serial capture on %s ...\n', UWB_TAG_COM);
                tu = dune.TagSerial(UWB_TAG_COM);
            end
        otherwise
            error('rover_teleop_uwb:badTransport', ...
                  'UWB_TRANSPORT must be "udp" or "serial", got "%s"', UWB_TRANSPORT);
    end
    tu.start();
    pause(1.0);
    tu.drain();                                  % flush anything stale
    fprintf('  listening (anchors: %s)\n', A.layout);
    if lower(string(UWB_TRANSPORT)) == "udp"
        fprintf('  NOTE: if no sweeps arrive, allow MATLAB through the Windows\n');
        fprintf('        firewall and check both tags are powered and on WiFi.\n\n');
    else
        fprintf('  NOTE: if only one tag id arrives, the slave has not joined -\n');
        fprintf('        check the UART cable and the SHARED GROUND, then watch\n');
        fprintf('        the WIRE,v1,... status lines for crc/resync counts.\n\n');
    end
end

%% ========================================
%% INITIALIZE PS4 CONTROLLER
%% ========================================

fprintf('Connecting PS4 controller...\n');
try
    joy = vrjoystick(1);
    fprintf('  PS4 controller connected\n\n');
catch
    error('PS4 controller not found. Connect via USB or Bluetooth and retry.');
end
[~, prev_buttons, prev_pov] = read(joy);

%% ========================================
%% PREPARE DATA STRUCTURE
%% ========================================

MAX_SAMPLES = 60000;      % ~50 min at 20 Hz
MAX_UWB     = 60000;

data = struct();
data.time = zeros(MAX_SAMPLES, 1);
data.t_posix = nan(MAX_SAMPLES, 1);      % absolute clock -> aligns with UWB
data.segment_id = zeros(MAX_SAMPLES, 1);
data.V_cmd = zeros(MAX_SAMPLES, 1);
data.omega_cmd = zeros(MAX_SAMPLES, 1);
data.apriltag_pos = nan(MAX_SAMPLES, 3);
data.apriltag_euler = nan(MAX_SAMPLES, 3);
data.imu_gyro = nan(MAX_SAMPLES, 3);
data.imu_accel = nan(MAX_SAMPLES, 3);
data.imu_euler = nan(MAX_SAMPLES, 3);
data.imu_accuracy = zeros(MAX_SAMPLES, 1);
data.motor_rpm = zeros(MAX_SAMPLES, 6);
data.wheel_rpm = zeros(MAX_SAMPLES, 6);
data.commanded_motor_rpm = zeros(MAX_SAMPLES, 6);
data.steering_angles = zeros(MAX_SAMPLES, 4);

% --- UWB block (DUNE addition) ---
% IMPORTANT: kept in its OWN variable, NOT inside `data`. logDataPointHW takes
% `data` by value and modifies it, so MATLAB copy-on-write duplicates the whole
% struct every iteration - burying ~13 MB of UWB arrays in there collapsed the
% control loop to ~1.4 Hz (PS4 presses were missed between polls). Merged into
% `data` once, at save time.
uwb = struct();
uwb.anchorIds = A.ids(:)';
uwb.t_posix = nan(MAX_UWB, 1);
uwb.t_rel   = nan(MAX_UWB, 1);      % seconds since test start
uwb.tag     = nan(MAX_UWB, 1);
uwb.R       = nan(MAX_UWB, uwbM);   % raw range per anchor (m)
uwb.RX      = nan(MAX_UWB, uwbM);
uwb.FP      = nan(MAX_UWB, uwbM);
uwb.quat    = nan(MAX_UWB, 4);      % tag-240 IMU tail (NaN for 241)
uwb.gyro    = nan(MAX_UWB, 3);
uwb.acc     = nan(MAX_UWB, 3);
uwb_idx = 1;

metadata = struct();
metadata.test_date = datetime('now');
metadata.control_rate = CONTROL_RATE;
metadata.ps4_controlled = true;
metadata.v_increment = V_INCREMENT;
metadata.omega_increment = OMEGA_INCREMENT;
metadata.wheel_radius = WHEEL_RADIUS;
metadata.gear_ratio = GEAR_RATIO;
metadata.wheelbase = WHEELBASE;
metadata.track_width = TRACK_WIDTH;
metadata.v_max = V_MAX;
metadata.omega_max = OMEGA_MAX;
metadata.hardware_interface = 'HardwareManipulatorControl';
metadata.truth_source = char(TRUTH_SOURCE);   % "dune" (calibrated) | "mms" (legacy)
if lower(TRUTH_SOURCE) == "dune" && ~isempty(visionCfg)
    metadata.truth_frame = 'uwb_anchor_frame (DUNE registered extrinsics)';
    metadata.truth_registration_rmse_m = visionCfg.extrinsics.rmse_m;
    metadata.truth_apriltag_size_m = visionCfg.apriltag.base.size;
else
    metadata.truth_frame = 'mms_world (legacy nadir)';
end
metadata.ported_from = ['D:\MMS_Codebase\matlab\slip_analysis\' ...
                        'test_random_excitation_slip_measurement.m'];
% DUNE/UWB metadata
metadata.uwb.enabled = UWB_ENABLE;
metadata.uwb.transport = char(UWB_TRANSPORT);   % "udp" | "serial" (wired tags)
metadata.uwb.port = UWB_PORT;
metadata.uwb.tag_com = char(UWB_TAG_COM);
metadata.uwb.frontTag = UWB_FRONT_TAG;
metadata.uwb.rearTag = UWB_REAR_TAG;
metadata.uwb.baseline_m = UWB_BASELINE;
metadata.uwb.apriltag_offset_body = APRILTAG_OFFSET_BODY;
metadata.uwb.anchors_file = A.file;
metadata.uwb.anchors_layout = A.layout;
metadata.uwb.frame_note = FRAME_NOTE;

%% ========================================
%% SETUP STATUS DISPLAY
%% ========================================

status_fig = figure('Name', 'Rover teleop + UWB capture', ...
    'Position', [50, 50, 440, 380], 'MenuBar', 'none', 'ToolBar', 'none', ...
    'NumberTitle', 'off', 'Color', [0.15, 0.15, 0.15], ...
    'UserData', struct('stop_requested', false, 'hw', hw));
set(status_fig, 'KeyPressFcn', @(src, event) checkKeyboardStopHW(src, event, hw));
set(status_fig, 'CloseRequestFcn', @(src, ~) triggerEmergencyStopHW(src, hw, true));

status_text = uicontrol('Style', 'text', 'Parent', status_fig, ...
    'Position', [10, 60, 420, 310], 'FontName', 'Courier New', 'FontSize', 11, ...
    'ForegroundColor', [0.0, 1.0, 0.0], 'BackgroundColor', [0.15, 0.15, 0.15], ...
    'HorizontalAlignment', 'left', 'String', 'Initializing...');

uicontrol('Style', 'pushbutton', 'Parent', status_fig, 'String', 'EMERGENCY STOP', ...
    'FontSize', 12, 'FontWeight', 'bold', 'ForegroundColor', 'white', ...
    'BackgroundColor', [0.8, 0, 0], 'Position', [10, 10, 420, 40], ...
    'Callback', @(~, ~) triggerEmergencyStopHW(status_fig, hw, false));

figure(status_fig); drawnow;

%% ========================================
%% MAIN CONTROL LOOP
%% ========================================

fprintf('=== READY - Use PS4 controller ===\n');
fprintf('D-pad: V (Up/Down), omega (Left/Right)\n');
fprintf('R1: Reverse | Circle: E-Stop | PS: Exit & Save\n\n');

currentV = 0; currentOmega = 0; reverseMode = false;
segment = 1; sample_idx = 1; loop_count = 0;
nUwbTag = containers.Map({UWB_FRONT_TAG, UWB_REAR_TAG}, {0, 0});
lastUwbT = NaN;
tSensors = 0; tUwb = 0;          % cumulative section timing (loop-rate diag)
lastRateT = 0; lastRateN = 0; loopHz = NaN;
inputN = 0; lastInputN = 0; inputHz = NaN;   % PS4 polling rate (the one that matters)
lastSensorT = -inf;              % when the heavy sensor block last ran
lastUwbDrainT = -inf;            % when the UDP socket was last drained
lastVelSent = [NaN NaN];         % last (V,omega) actually transmitted
lastVelT = -inf;                 % ... and when (for the keepalive)

test_start_time = tic;
test_start_posix = posixtime(datetime('now', 'TimeZone', 'UTC'));
last_status_print = tic;
last_display_update = tic;
running = true;

try
    while running
        loop_count = loop_count + 1;
        current_time = toc(test_start_time);

        if ~isvalid(status_fig) || checkEmergencyStopHW(status_fig)
            fprintf('\n  Figure emergency stop triggered\n');
            break;
        end

        %% --- Read PS4 Controller ---
        [~, buttons, pov] = read(joy);
        if length(buttons) < 8
            pause(1/CONTROL_RATE);
            continue;
        end
        btn_rising = buttons & ~prev_buttons;
        old_V = currentV; old_omega = currentOmega; old_reverse = reverseMode;

        %% --- D-pad: V and omega (debounced on state change) ---
        % NOTE: kept byte-identical to the known-good original. Do not
        % "improve" this while the port is still being debugged.
        if pov ~= prev_pov && pov >= 0
            if pov == 0
                currentV = min(currentV + V_INCREMENT, V_MAX);
                fprintf('  V+ = %.3f m/s\n', currentV);
            elseif pov == 180
                currentV = max(currentV - V_INCREMENT, 0);
                fprintf('  V- = %.3f m/s\n', currentV);
            elseif pov == 270
                currentOmega = min(currentOmega + OMEGA_INCREMENT, OMEGA_MAX);
                fprintf('  omega+ = %.3f rad/s (left)\n', currentOmega);
            elseif pov == 90
                currentOmega = max(currentOmega - OMEGA_INCREMENT, -OMEGA_MAX);
                fprintf('  omega- = %.3f rad/s (right)\n', currentOmega);
            end
        end

        %% --- Button actions (rising edge) ---
        if btn_rising(6)
            reverseMode = ~reverseMode;
            fprintf('  REVERSE MODE: %s\n', string(reverseMode));
        end
        if btn_rising(3)
            currentV = 0; currentOmega = 0; reverseMode = false;
            hw.stopAll();
            fprintf('  !!! EMERGENCY STOP !!!\n');
        end
        if btn_rising(4)
            currentV = 0; currentOmega = 0;
            fprintf('  RESET: V=0, omega=0\n');
        end
        if btn_rising(1)
            currentOmega = 0;
            fprintf('  STRAIGHT: omega=0\n');
        end
        if btn_rising(7)
            fprintf('\n--- CURRENT STATE ---\n');
            fprintf('  V:       %.3f m/s\n', currentV);
            fprintf('  omega:   %.3f rad/s (%.1f deg/s)\n', currentOmega, rad2deg(currentOmega));
            fprintf('  Reverse: %s\n', string(reverseMode));
            fprintf('  Segment: %d\n', segment);
            fprintf('  Samples: %d   UWB sweeps: %d\n', sample_idx - 1, uwb_idx - 1);
            fprintf('  Time:    %.1f s\n', current_time);
            fprintf('-------------------\n\n');
        end
        if length(buttons) >= 13 && btn_rising(13)
            fprintf('\n  PS button pressed - exiting...\n');
            running = false;
        end

        %% --- Track segment changes ---
        if round(currentV, 4) ~= round(old_V, 4) || ...
           round(currentOmega, 4) ~= round(old_omega, 4) || ...
           reverseMode ~= old_reverse
            segment = segment + 1;
        end

        %% --- Compute and send velocity command ---
        V_cmd = currentV;
        if reverseMode, V_cmd = -V_cmd; end
        omega_cmd = currentOmega;
        inputN = inputN + 1;

        %% --- Send velocity ON CHANGE (+ keepalive), not every iteration ---
        % setRoverVelocity costs ~5 serial round-trips (~100-150 ms). The
        % motors latch their last command (the original relied on this while
        % idle), so re-sending an unchanged command every loop only burned
        % time. Changes go out immediately; a 1 s keepalive re-asserts.
        velChanged = ~isequal([V_cmd, omega_cmd], lastVelSent);
        if abs(V_cmd) > 0.001 || abs(omega_cmd) > 0.001
            if velChanged || toc(test_start_time) - lastVelT > VEL_KEEPALIVE
                hw.setRoverVelocity(V_cmd, omega_cmd);
                lastVelSent = [V_cmd, omega_cmd];
                lastVelT = toc(test_start_time);
            end
        else
            if velChanged || toc(test_start_time) - lastVelT > VEL_KEEPALIVE
                hw.stopAll();
                lastVelSent = [V_cmd, omega_cmd];
                lastVelT = toc(test_start_time);
            end
        end

        %% --- Log rover data on ITS OWN schedule (~3 Hz), not per-iteration ---
        % The AprilTag detect alone is 150-280 ms; running it every loop is
        % what made button presses vanish. The logged data rate is unchanged
        % (the hardware never delivered more than ~3 Hz anyway).
        if sample_idx <= MAX_SAMPLES && ...
                current_time - lastSensorT >= SENSOR_PERIOD
            lastSensorT = current_time;
            % fast absolute clock: start + elapsed. posixtime(datetime(...,TZ))
            % costs ~0.6 ms per call, which is dead weight in a 20 Hz loop.
            data.t_posix(sample_idx) = test_start_posix + current_time;
            tS = tic;
            [data, sample_idx] = logDataPointHW(hw, data, sample_idx, current_time, ...
                segment, V_cmd, omega_cmd, GEAR_RATIO, ...
                APRILTAG_RETRIES, MAX_RETRIES, RETRY_DELAY, ...
                TRUTH_SOURCE, visionCfg, APRILTAG_ID);
            tSensors = tSensors + toc(tS);
        end

        %% --- Drain UWB on a timer (an EMPTY Java poll costs ~15 ms) ---
        if UWB_ENABLE && current_time - lastUwbDrainT >= UWB_DRAIN_PERIOD
            lastUwbDrainT = current_time;
            tU = tic;
            for c = tu.drain()
                s = c{1};
                if uwb_idx > MAX_UWB, break; end
                if ~ismember(s.tag, [UWB_FRONT_TAG, UWB_REAR_TAG]), continue; end
                uwb.t_posix(uwb_idx) = s.thost;
                uwb.t_rel(uwb_idx)   = s.thost - test_start_posix;
                uwb.tag(uwb_idx)     = s.tag;
                for m = 1:numel(s.ids)
                    col = find(A.ids == s.ids(m), 1);
                    if isempty(col), continue; end
                    uwb.R(uwb_idx, col)  = s.dist(m);
                    uwb.RX(uwb_idx, col) = s.rx(m);
                    uwb.FP(uwb_idx, col) = s.fp(m);
                end
                if ~isempty(s.imu)
                    uwb.quat(uwb_idx, :) = s.imu.quat(:)';
                    uwb.gyro(uwb_idx, :) = s.imu.gyro(:)';
                    uwb.acc(uwb_idx, :)  = s.imu.acc(:)';
                end
                if nUwbTag.isKey(s.tag), nUwbTag(s.tag) = nUwbTag(s.tag) + 1; end
                lastUwbT = current_time;
                uwb_idx = uwb_idx + 1;
            end
            % drainEvents() pumps the socket a SECOND time (~15 ms each, the
            % cost is a Java socket-timeout exception), so only flush the event
            % queue occasionally - drain() above already collected the sweeps.
            if mod(loop_count, 40) == 0, tu.drainEvents(); end
            tUwb = tUwb + toc(tU);
        end

        %% --- Update status display ---
        if toc(last_display_update) >= DISPLAY_UPDATE_INTERVAL
            if isvalid(status_fig)
                rev_str = ''; if reverseMode, rev_str = '  ** REVERSE **'; end
                if UWB_ENABLE
                    age = NaN; if isfinite(lastUwbT), age = current_time - lastUwbT; end
                    uwbStr = sprintf('UWB %d: F%d R%d  (last %.1fs)', uwb_idx - 1, ...
                        nUwbTag(UWB_FRONT_TAG), nUwbTag(UWB_REAR_TAG), age);
                    if ~isfinite(age) || age > 3
                        uwbStr = [uwbStr ' <-- STALE!'];
                    end
                else
                    uwbStr = 'UWB: disabled';
                end
                status_str = sprintf([ ...
                    'V:     %+.3f m/s%s\n' ...
                    'omega: %+.3f rad/s\n\n' ...
                    'Segment: %d\n' ...
                    'Samples: %d\n' ...
                    '%s\n' ...
                    'Time:    %.1f s\n\n' ...
                    'D-pad Up/Dn:  V +/- %.2f\n' ...
                    'D-pad L/R:    omega +/- %.2f\n' ...
                    'R1: Reverse  | Sq: Straight\n' ...
                    'Circle: E-Stop | Tri: Reset\n' ...
                    'PS: Exit & Save'], ...
                    V_cmd, rev_str, omega_cmd, segment, sample_idx - 1, ...
                    uwbStr, current_time, V_INCREMENT, OMEGA_INCREMENT);
                set(status_text, 'String', status_str);
                if reverseMode
                    set(status_text, 'ForegroundColor', [1.0, 0.3, 0.3]);
                elseif abs(V_cmd) < 0.001 && abs(omega_cmd) < 0.001
                    set(status_text, 'ForegroundColor', [1.0, 1.0, 0.3]);
                else
                    set(status_text, 'ForegroundColor', [0.0, 1.0, 0.0]);
                end
            end
            last_display_update = tic;
        end

        %% --- Console status print ---
        if toc(last_status_print) >= STATUS_PRINT_INTERVAL
            rev_tag = ''; if reverseMode, rev_tag = ' [REV]'; end
            % Achieved rates. inputHz is the one that matters for feel: PS4
            % presses are edge-detected between polls, so input polling much
            % below ~10 Hz silently drops button taps. sensor rate is the
            % logging schedule (~1/SENSOR_PERIOD by design).
            dN = (sample_idx - 1) - lastRateN;
            dT = current_time - lastRateT;
            if dT > 0
                loopHz = dN / dT;
                inputHz = (inputN - lastInputN) / dT;
            end
            lastRateN = sample_idx - 1; lastRateT = current_time;
            lastInputN = inputN;
            warnStr = '';
            if isfinite(inputHz) && inputHz < 8
                warnStr = '  <-- SLOW INPUT: PS4 presses will be missed';
            end
            fprintf('[%5.1fs] V=%+.3f omega=%+.3f seg=%d n=%d uwb=%d input %.1fHz sens %.1fHz (%.0f%%)%s%s\n', ...
                current_time, V_cmd, omega_cmd, segment, sample_idx - 1, ...
                uwb_idx - 1, inputHz, loopHz, ...
                100*tSensors/max(current_time,1e-3), rev_tag, warnStr);
            last_status_print = tic;
        end

        prev_buttons = buttons;
        prev_pov = pov;
        pause(1/CONTROL_RATE);
        drawnow limitrate;
    end

catch ME
    fprintf('\n!!! TEST INTERRUPTED !!!\n');
    fprintf('Error: %s\n', ME.message);
end

%% ========================================
%% CLEANUP
%% ========================================

hw.stopAll();
pause(0.5);
if UWB_ENABLE && ~isempty(tu), delete(tu); end
if isvalid(status_fig)
    set(status_fig, 'CloseRequestFcn', 'closereq');
    close(status_fig);
end

%% ========================================
%% DATA QUALITY REPORT
%% ========================================

total_samples = sample_idx - 1;
total_uwb = uwb_idx - 1;
fprintf('\n--- Data Quality Report ---\n');
fprintf('Rover samples: %d | segments: %d\n', total_samples, segment);

if total_samples == 0
    fprintf('No data collected!\n');
else
    elapsed = data.time(total_samples);
    fprintf('Duration: %.1f s | avg rate: %.1f Hz\n', elapsed, total_samples/max(elapsed,0.01));
    apriltag_valid = sum(~isnan(data.apriltag_pos(1:total_samples, 1)));
    imu_valid = sum(~isnan(data.imu_gyro(1:total_samples, 1)));
    encoder_valid = sum(any(data.motor_rpm(1:total_samples, :) ~= 0, 2));
    fprintf('  AprilTag: %d/%d (%.1f%%)\n', apriltag_valid, total_samples, 100*apriltag_valid/total_samples);
    fprintf('  Base IMU: %d/%d (%.1f%%)\n', imu_valid, total_samples, 100*imu_valid/total_samples);
    fprintf('  Encoders: %d/%d (%.1f%%)\n', encoder_valid, total_samples, 100*encoder_valid/total_samples);
    if apriltag_valid < total_samples * 0.5
        fprintf('\n  WARNING: AprilTag detection < 50%% - truth will be sparse.\n');
    end
end

if UWB_ENABLE
    fprintf('UWB sweeps: %d  (front %d=%d, rear %d=%d)\n', total_uwb, ...
        UWB_FRONT_TAG, nUwbTag(UWB_FRONT_TAG), UWB_REAR_TAG, nUwbTag(UWB_REAR_TAG));
    if total_uwb > 0 && total_samples > 0
        fprintf('  UWB rate: %.1f Hz total\n', total_uwb / max(data.time(total_samples), 0.01));
        nAnch = sum(~isnan(uwb.R(1:total_uwb, :)), 2);
        fprintf('  anchors/sweep: mean %.1f of %d\n', mean(nAnch), uwbM);
    end
    if nUwbTag(UWB_FRONT_TAG) == 0 || nUwbTag(UWB_REAR_TAG) == 0
        fprintf('\n  WARNING: one tag never reported - the rigid solve needs BOTH.\n');
    end
end

%% ========================================
%% SAVE DATA
%% ========================================

if total_samples > 0
    f = {'time','t_posix','segment_id','V_cmd','omega_cmd'};
    for k = 1:numel(f), data.(f{k}) = data.(f{k})(1:total_samples); end
    f2 = {'apriltag_pos','apriltag_euler','imu_gyro','imu_accel','imu_euler', ...
          'motor_rpm','wheel_rpm','commanded_motor_rpm','steering_angles'};
    for k = 1:numel(f2), data.(f2{k}) = data.(f2{k})(1:total_samples, :); end
    data.imu_accuracy = data.imu_accuracy(1:total_samples);

    uf = {'t_posix','t_rel','tag'};
    for k = 1:numel(uf), uwb.(uf{k}) = uwb.(uf{k})(1:total_uwb); end
    uf2 = {'R','RX','FP','quat','gyro','acc'};
    for k = 1:numel(uf2), uwb.(uf2{k}) = uwb.(uf2{k})(1:total_uwb, :); end
    data.uwb = uwb;      % merge ONCE, after the loop (see the note at its init)

    metadata.data_quality.total_samples = total_samples;
    metadata.data_quality.total_segments = segment;
    metadata.data_quality.duration_sec = data.time(end);
    metadata.data_quality.avg_rate_hz = total_samples / max(data.time(end), 0.01);
    metadata.data_quality.total_uwb_sweeps = total_uwb;
    metadata.test_start_posix = test_start_posix;

    outDir = fullfile(dune.rootDir(), 'results', 'rover_runs');
    if ~exist(outDir, 'dir'), mkdir(outDir); end
    stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    filepath = fullfile(outDir, sprintf('rover_uwb_%s.mat', stamp));

    fprintf('\nSaving to: %s\n', filepath);
    save(filepath, 'data', 'metadata', '-v7.3');
    fprintf('  Saved (%d rover samples, %d UWB sweeps, %.1f s)\n', ...
            total_samples, total_uwb, data.time(end));
end

delete(hw);
fprintf('\n=== SESSION COMPLETE ===\n\n');

%% ========================================
%% OFFLINE REPORT
%% ========================================
% Runs the estimator over the sweeps we just logged and compares against the
% AprilTag truth (frames are logged RAW - the report fits the 2D transform).
% Hardware is already released above, so a failure here costs nothing.

if total_samples > 0
    fprintf('Generating run report...\n');
    try
        rover_run_report(filepath);
    catch ME
        fprintf(2, 'report failed: %s\n', ME.message);
        fprintf(2, 'run it by hand:  rover_run_report(''%s'')\n', filepath);
    end
end

%% ========================================
%% HELPER FUNCTIONS (ported unchanged)
%% ========================================

function [data, idx] = logDataPointHW(hw, data, idx, time, seg_id, V_cmd, omega_cmd, ...
                                       gear_ratio, apriltag_retries, max_retries, retry_delay, ...
                                       truth_source, vision_cfg, apriltag_id)
    data.time(idx) = time;
    data.segment_id(idx) = seg_id;
    data.V_cmd(idx) = V_cmd;
    data.omega_cmd(idx) = omega_cmd;

    % AprilTag: its OWN retry budget (default 1). Each attempt is a full
    % frame grab + detection (~150-280 ms), so the old 3-retry policy cost
    % 500-900 ms whenever the marker was out of view. Missed samples are
    % interpolated offline; a stalled control loop is not recoverable.
    % Source (DUNE calibrated vs MMS legacy) is chosen by TRUTH_SOURCE.
    [roverPos, roverOri, det] = getTruthPose(hw, truth_source, vision_cfg, ...
                                             apriltag_id, apriltag_retries, retry_delay);
    if det
        data.apriltag_pos(idx, :) = roverPos;
        data.apriltag_euler(idx, :) = roverOri;
    end

    for retry = 1:max_retries
        try
            imu_data = hw.getTeensyIMU();
            if ~isempty(imu_data) && isfield(imu_data, 'available') && imu_data.available
                data.imu_gyro(idx, :) = [imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z];
                data.imu_accel(idx, :) = [imu_data.accel_x, imu_data.accel_y, imu_data.accel_z];
                data.imu_euler(idx, :) = [imu_data.roll, imu_data.pitch, imu_data.yaw];
                data.imu_accuracy(idx) = imu_data.accuracy;
                break;
            end
        catch
            if retry < max_retries, pause(retry_delay); end
        end
    end

    for retry = 1:max_retries
        try
            motor_rpm = hw.getEncoderSpeeds();
            if ~isempty(motor_rpm) && length(motor_rpm) == 6
                data.motor_rpm(idx, :) = motor_rpm(:)';
                data.wheel_rpm(idx, :) = motor_rpm(:)' / gear_ratio;
                break;
            end
        catch
            if retry < max_retries, pause(retry_delay); end
        end
    end

    WHEELBASE = 0.6; TRACK_WIDTH = 0.5; MAX_STEERING = 40;
    if abs(omega_cmd) < 1e-6
        data.steering_angles(idx, :) = [0, 0, 0, 0];
    else
        turnRadius = abs(V_cmd) / abs(omega_cmd);
        MIN_TURN_RADIUS = WHEELBASE / tand(MAX_STEERING);
        turnRadius = max(turnRadius, MIN_TURN_RADIUS);
        steerAngle = atand(WHEELBASE / turnRadius);
        if omega_cmd < 0, steerAngle = -steerAngle; end
        if abs(steerAngle) < 0.1
            data.steering_angles(idx, :) = [0, 0, 0, 0];
        else
            innerRadius = turnRadius - TRACK_WIDTH/2;
            outerRadius = turnRadius + TRACK_WIDTH/2;
            innerAngle = min(atand(WHEELBASE / innerRadius), MAX_STEERING);
            outerAngle = min(atand(WHEELBASE / outerRadius), MAX_STEERING);
            if steerAngle > 0
                data.steering_angles(idx, :) = [innerAngle, innerAngle, outerAngle, outerAngle];
            else
                data.steering_angles(idx, :) = [-outerAngle, -outerAngle, -innerAngle, -innerAngle];
            end
        end
    end
    idx = idx + 1;
end

function stop_requested = checkEmergencyStopHW(fig)
    if ~isvalid(fig), stop_requested = true; return; end
    ud = get(fig, 'UserData');
    stop_requested = ud.stop_requested;
end

function triggerEmergencyStopHW(fig, hw, close_after)
    if nargin < 3, close_after = false; end
    fprintf('\n!!! EMERGENCY STOP TRIGGERED !!!\n');
    try
        hw.stopAll();
        fprintf('  Rover stopped\n');
    catch
        fprintf('Warning: Could not stop hardware\n');
    end
    if isvalid(fig)
        ud = get(fig, 'UserData');
        ud.stop_requested = true;
        set(fig, 'UserData', ud);
        btn = findobj(fig, 'Style', 'pushbutton');
        if ~isempty(btn)
            set(btn, 'String', 'STOPPED', 'BackgroundColor', [0.3, 0.3, 0.3]);
        end
        if close_after
            set(fig, 'CloseRequestFcn', 'closereq');
            close(fig);
        end
    end
end

function checkKeyboardStopHW(fig, event, hw)
    key = event.Key;
    if strcmpi(key, 'space') || strcmpi(key, 'escape') || strcmpi(key, 'q')
        triggerEmergencyStopHW(fig, hw, false);
    end
end

function [pos, ori, det] = getTruthPose(hw, source, visionCfg, tagId, retries, retryDelay)
    %GETTRUTHPOSE One AprilTag world pose from the selected ground-truth source.
    %   "dune" - hw.getColorFrame() (read-only MMS call) + DUNE calibrated
    %            vision (getAprilTagPoseFromFrame): truth in the UWB anchor frame.
    %   "mms"  - hw.getRoverPose(): legacy MMS intrinsics + nadir transform.
    %   Returns pos [1x3] / ori [1x3] as row vectors and det (logical). Both
    %   sources cost one Kinect frame grab per attempt.
    pos = [nan nan nan]; ori = [nan nan nan]; det = false;
    for r = 1:retries
        try
            if lower(string(source)) == "dune"
                rgb = hw.getColorFrame();
                [p, o, ~, isDet] = getAprilTagPoseFromFrame(rgb, visionCfg, tagId);
                if isDet, pos = p(:)'; ori = o(:)'; det = true; return; end
            else
                [p, o] = hw.getRoverPose();
                if ~isempty(p), pos = p(:)'; ori = o(:)'; det = true; return; end
            end
        catch
            % swallow: a missed detection is interpolated offline; never stall
        end
        if r < retries, pause(retryDelay); end
    end
end
