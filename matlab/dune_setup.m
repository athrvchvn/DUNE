function dune_setup()
%DUNE_SETUP Put the DUNE MATLAB pipeline on the path. Run this first.
%
%   >> cd matlab
%   >> dune_setup
%
%   Adds matlab/ (so the `dune.*` package resolves), matlab/scripts and
%   matlab/vision, then reports what it found. Nothing is written and nothing
%   is changed permanently — run it once per MATLAB session, or add the line
%   to your startup.m.
%
%   The moving-horizon estimators additionally need CasADi; dune.addCasadi
%   locates it and dune.addCasadi_available reports whether it is usable. The
%   rest of the pipeline — parsing, calibration correction, NLOS weighting,
%   multilateration, the EKF — runs without it.

here = fileparts(mfilename('fullpath'));
addpath(here);                                  % makes `dune.*` resolvable
addpath(fullfile(here, 'scripts'));
addpath(fullfile(here, 'vision'));

fprintf('DUNE MATLAB pipeline\n');
fprintf('  repo root : %s\n', dune.rootDir());
fprintf('  config    : %s\n', dune.configDir());

anchorsFile = dune.configDir('anchors.json');
if isfile(anchorsFile)
    try
        A = dune.loadAnchors();
        fprintf('  anchors   : %d loaded from anchors.json\n', numel(A.ids));
    catch err
        fprintf(2, '  anchors   : anchors.json present but unreadable (%s)\n', ...
                err.message);
    end
else
    fprintf(2, ['  anchors   : config/anchors.json missing. Copy\n' ...
                '              config/anchors.example.json and edit it, or\n' ...
                '              run the self-survey (docs/anchor_survey.md).\n']);
end

if dune.addCasadi_available()
    fprintf('  CasADi    : available (MHE enabled)\n');
else
    fprintf(['  CasADi    : not found. The EKF path still runs; the\n' ...
             '              moving-horizon estimators do not. See\n' ...
             '              docs/matlab_pipeline.md.\n']);
end

fprintf(['\nNext: live_tag (single tag) or live_rover (dual-tag rigid MHE).\n' ...
         'Offline: mhe_replay, rover_run_report, replay_rtls.\n']);
end
