function d = logsDir()
%LOGSDIR Path to logs/ — where a live session records. Created on demand.
%
%   See also DUNE.ROOTDIR, DUNE.RESULTSDIR.
d = fullfile(dune.rootDir(), 'logs');
if ~isfolder(d), mkdir(d); end
end
