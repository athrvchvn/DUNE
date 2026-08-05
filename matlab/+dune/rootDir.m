function d = rootDir()
%ROOTDIR Absolute path of the repository root (the folder holding matlab/).
%
%   Everything shared between the two host pipelines hangs off this:
%
%       fullfile(dune.rootDir(), 'config')    anchor geometry, corrections,
%                                             camera calibration  (tracked)
%       fullfile(dune.rootDir(), 'logs')      session recordings  (gitignored)
%       fullfile(dune.rootDir(), 'results')   report output       (gitignored)
%       fullfile(dune.rootDir(), 'data')      datasets, distributed separately
%
%   config/ deliberately sits at the repository root rather than under matlab/:
%   the Python pipeline reads exactly the same files. Configuration describes
%   the deployment, not one implementation of it.
%
%   See also DUNE.CONFIGDIR, DUNE.DATADIR, DUNE.LOGSDIR, DUNE.RESULTSDIR.

% +dune/rootDir.m -> +dune -> matlab -> repo root
d = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end
