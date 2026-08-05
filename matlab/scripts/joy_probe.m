function joy_probe(seconds, idx)
%JOY_PROBE Find the joystick and discover its ACTUAL button/POV mapping.
%   joy_probe            % auto-scan indices 1..4, then watch for 30 s
%   joy_probe(60)        % watch for 60 s
%   joy_probe(30, 2)     % force joystick index 2
%
%   Why: PS4 pads enumerate differently over USB vs Bluetooth, and Steam or
%   DS4Windows can remap them to an Xbox layout. rover_teleop_uwb assumes the
%   DirectInput layout (R1=6, Circle=3, Triangle=4, Square=1, L2=7, PS=13) and
%   SILENTLY skips its whole loop if the pad reports fewer than 8 buttons -
%   which looks exactly like "the controller does nothing".
%
%   Press each control in turn; this prints the index it really reports. Send
%   me those numbers and I will remap rover_teleop_uwb accordingly.

arguments
    seconds (1,1) double = 30
    idx (1,1) double = 0        % 0 = auto-scan
end

fprintf('\n=== joystick probe ===\n');

if idx == 0
    ok = [];
    for k = 1:4
        try
            j = vrjoystick(k);
            [a, b, p] = read(j);
            fprintf('  index %d: %2d axes, %2d buttons, POV = %s\n', ...
                    k, numel(a), numel(b), mat2str(p));
            ok(end+1) = k; %#ok<AGROW>
            close(j);
        catch
            fprintf('  index %d: not available\n', k);
        end
    end
    if isempty(ok)
        error(['No joystick found.\n' ...
               '  - try USB instead of Bluetooth (or vice versa)\n' ...
               '  - close Steam / DS4Windows (they capture the pad)\n' ...
               '  - check Windows "Set up USB game controllers" sees it']);
    end
    idx = ok(1);
    fprintf('  -> using index %d\n', idx);
end

joy = vrjoystick(idx);
cleanup = onCleanup(@() close(joy)); %#ok<NASGU>
[a0, b0, p0] = read(joy);

fprintf('\n%d axes, %d buttons.\n', numel(a0), numel(b0));
if numel(b0) < 8
    fprintf(2, ['WARNING: only %d buttons. rover_teleop_uwb requires >= 8 and\n' ...
                '         will silently do nothing. This is your problem.\n'], numel(b0));
end
fprintf(['\nNow press, one at a time, and note the index printed:\n' ...
         '   D-pad Up / Down / Left / Right   (shows as POV)\n' ...
         '   R1, Circle, Triangle, Square, L2, PS button\n' ...
         'Watching for %.0f s (Ctrl-C to stop early)...\n\n'], seconds);

t0 = tic; prevB = b0; prevP = p0; prevA = a0;
while toc(t0) < seconds
    [a, b, p] = read(joy);

    for r = find(b & ~prevB)
        fprintf('[%5.1fs]  BUTTON %d  pressed\n', toc(t0), r);
    end
    if ~isequal(p, prevP)
        fprintf('[%5.1fs]  POV = %s\n', toc(t0), mat2str(p));
    end
    if numel(a) == numel(prevA)
        for g = find(abs(a - prevA) > 0.35)
            fprintf('[%5.1fs]  AXIS %d = %+.2f\n', toc(t0), g, a(g));
        end
    end

    prevB = b; prevP = p; prevA = a;
    pause(0.05);
end

fprintf('\nDone. Send me the indices and I will remap rover_teleop_uwb.\n');
end
