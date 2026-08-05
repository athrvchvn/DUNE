function d = configDir(name)
%CONFIGDIR Path to config/, or to a named file inside it.
%
%   dune.configDir()                 -> <repo>/config
%   dune.configDir('anchors.json')   -> <repo>/config/anchors.json
%
%   The same directory the Python pipeline reads. Existence is the caller's
%   problem — several callers legitimately probe for an optional file.
%
%   See also DUNE.ROOTDIR.
d = fullfile(dune.rootDir(), 'config');
if nargin > 0
    d = fullfile(d, name);
end
end
