function replay_log(logPath, anchorsPath)
%REPLAY_LOG Parse a recorded session and solve every sweep — MATLAB side.
%
%   replay_log                              % bundled example log
%   replay_log("path/to/serial_raw.log")
%   replay_log(logPath, dune.configDir("anchors.json"))
%
%   Deliberately mirrors examples/replay_log.py. The two pipelines share the
%   wire format, config/ and the algorithms, so the numbers should agree — that
%   is the claim, and this is the short demonstration of it. Run both on one
%   log and compare.
%
%   Run dune_setup first, from the matlab/ folder.

arguments
    logPath     (1,1) string = fullfile(dune.rootDir(), "examples", "logs", "example_session.log")
    anchorsPath (1,1) string = dune.configDir("anchors_kinect_clicked_20260724.json")
end

if ~isfile(logPath)
    fprintf("No log at %s\n\n", logPath);
    fprintf("The bundled example log has not been added yet — see\n");
    fprintf("  examples/logs/README.md   and   docs/datasets.md\n\n");
    fprintf("Point this at any capture you have:\n");
    fprintf("  replay_log(""path/to/serial_raw.log"")\n");
    return
end

A  = dune.loadAnchors(anchorsPath);
rc = dune.loadRangeCorrection();
if isempty(rc), tier2 = "absent"; else, tier2 = "loaded"; end
fprintf("log     %s\n", logPath);
fprintf("anchors %s (%d anchors)\n", anchorsPath, numel(A.ids));
fprintf("Tier-2  %s\n\n", tier2);

txt   = string(splitlines(fileread(logPath)));
tags  = containers.Map('KeyType', 'int32', 'ValueType', 'any');
nLine = 0;

for k = 1:numel(txt)
    i = strfind(txt(k), "RTLS,");
    if isempty(i), continue; end
    nLine = nLine + 1;

    s = dune.parseRtlsLine(extractAfter(txt(k), i(1) - 1));
    if isempty(s), continue; end

    [pos, info] = dune.solveSweep(s, A, rangeCorr=rc, tagZ=0.24);
    if any(~isfinite(pos)), continue; end

    key = int32(s.tag);
    if ~isKey(tags, key), tags(key) = []; end
    tags(key) = [tags(key); pos(1), pos(2), info.rmse];
end

fprintf("%d RTLS lines\n\n", nLine);
if isempty(tags.Count) || tags.Count == 0
    fprintf("No sweeps solved. Wrong anchors file for this capture?\n");
    return
end

worst = 0;
for key = sort(cell2mat(keys(tags)))
    M    = tags(key);
    rmse = M(:,3) * 1000;
    worst = max(worst, median(rmse));
    fprintf("  tag %d\n", key);
    fprintf("    solved sweeps    %d\n", size(M,1));
    fprintf("    residual RMSE    median %.1f mm, 95th %.1f mm\n", ...
            median(rmse), prctile(rmse, 95));
    fprintf("    x  %6.2f .. %6.2f m\n", min(M(:,1)), max(M(:,1)));
    fprintf("    y  %6.2f .. %6.2f m\n", min(M(:,2)), max(M(:,2)));
end

fprintf("\n  Residual RMSE is how well the ranges agree with each other given the\n");
fprintf("  geometry — not distance from truth. Scoring against truth needs\n");
fprintf("  rover_run_report. See docs/matlab_pipeline.md.\n");

if worst > 300
    fprintf("\n  [!] Residual is high (%.0f mm). The ranges disagree with each other by\n", worst);
    fprintf("      far more than the radio's noise, which almost always means the anchors\n");
    fprintf("      file does not describe the constellation this log was recorded in.\n");
end
end
