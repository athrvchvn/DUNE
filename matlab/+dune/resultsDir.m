function d = resultsDir()
%RESULTSDIR Path to results/ — where reports are written. Created on demand.
%
%   Gitignored: this is output, and it is reproducible from logs/ and data/.
%
%   See also DUNE.ROOTDIR, DUNE.LOGSDIR.
d = fullfile(dune.rootDir(), 'results');
if ~isfolder(d), mkdir(d); end
end
