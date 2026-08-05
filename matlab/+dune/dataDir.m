function d = dataDir(name)
%DATADIR Path to data/, or to a named entry inside it.
%
%   Datasets are distributed separately from the source (see docs/datasets.md)
%   and unpack here. A fresh clone has only the placeholder README, so anything
%   reading from this location must tolerate its absence.
%
%   See also DUNE.ROOTDIR.
d = fullfile(dune.rootDir(), 'data');
if nargin > 0
    d = fullfile(d, name);
end
end
