function restore_anchor_delays(port, calibFile, opts)
%RESTORE_ANCHOR_DELAYS Push saved per-anchor antenna delays back into NVS.
%   restore_anchor_delays("COM12")                       % newest backup
%   restore_anchor_delays("COM12", "<path to delay_calibration.json>")
%   restore_anchor_delays("COM12", "", tag=true)         % also restore tag 240
%
%   Undoes a calibration that went wrong: reads the delay_ticks recorded in a
%   delay_calibration.json (a pre-calibration backup) and re-pushes each anchor's
%   value over UWB via SETANTDELAY (anchor_delay_tool), so the anchors return to
%   exactly the state they were in when the backup was taken. Anchors persist the
%   value in NVS, so this survives reboot and reflash.
%
%   With tag=true it also restores the reference tag's own delay (SETMYDELAY)
%   from reference_tag_delay_ticks.
%
%   Requires all anchors powered and the tag on `port` (Serial Monitor closed).

arguments
    port string
    calibFile string = ""
    opts.tag (1,1) logical = false
    opts.tagId (1,1) double = 240
end

% Default to the newest pre-calibration backup.
if calibFile == ""
    d = dir(fullfile(dune.rootDir(), 'config', 'backup_precalib_*', 'delay_calibration.json'));
    assert(~isempty(d), 'No backup_precalib_*/delay_calibration.json found - pass the path.');
    [~, i] = max([d.datenum]);
    calibFile = fullfile(d(i).folder, d(i).name);
end
fprintf('Restoring from: %s\n', calibFile);

C = jsondecode(fileread(calibFile));
an = C.anchors; if iscell(an), an = [an{:}]; end
ids = [an.id]; ticks = [an.delay_ticks];

fprintf('Will push these saved delays:\n');
for k = 1:numel(ids)
    fprintf('  A%d -> %d ticks\n', ids(k), ticks(k));
end
if opts.tag && isfield(C, 'reference_tag_delay_ticks')
    fprintf('  tag %d -> %d ticks (SETMYDELAY)\n', opts.tagId, C.reference_tag_delay_ticks);
end

% anchor_delay_tool pushes ONE tick value to a list of ids, so group anchors by
% value and issue one call per distinct value (usually one call per anchor).
[uv, ~, grp] = unique(ticks);
for g = 1:numel(uv)
    theseIds = ids(grp == g);
    fprintf('\n-- pushing %d ticks to anchor(s) %s --\n', uv(g), mat2str(theseIds));
    anchor_delay_tool(port, ids=theseIds, ticks=uv(g));
end

if opts.tag && isfield(C, 'reference_tag_delay_ticks')
    fprintf('\n-- restoring tag %d own delay --\n', opts.tagId);
    anchor_delay_tool(port, ids=[], tagId=opts.tagId, ...
                      tagTicks=C.reference_tag_delay_ticks);
end

fprintf('\nRestore complete. Verify with: calibrate_delays("%s", verifyOnly=true)\n', port);
end
