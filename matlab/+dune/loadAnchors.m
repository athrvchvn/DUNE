function A = loadAnchors(jsonPath)
%LOADANCHORS Load anchor geometry from anchors.json.
%   A.ids   [M x 1]  anchor short addresses (1..5)
%   A.pos   [M x 3]  anchor antenna positions (m), world/anchor frame
%   A.bounds [xmin xmax ymin ymax zmin zmax]
%   A.dim   configured solve dimension (2 -> planar solve, fixed tag z)

if nargin < 1 || isempty(jsonPath)
    jsonPath = fullfile(dune.rootDir(), 'config', 'anchors.json');
end
raw = jsondecode(fileread(jsonPath));

n = numel(raw.anchors);
A.ids = zeros(n, 1);
A.pos = zeros(n, 3);
for k = 1:n
    a = raw.anchors(k);
    A.ids(k)   = a.id;
    A.pos(k,:) = [a.x, a.y, a.z];
end
A.bounds = raw.bounds(:)';
A.dim    = raw.dim;
% `layout` is a free-text provenance string, not geometry. Older anchors.json
% files (and anchors.example.json) predate it, and pointing replay_rtls at one
% of those must not fail on a missing label -- so default it rather than index
% into a struct that has no such field.
if isfield(raw, 'layout')
    A.layout = raw.layout;
else
    A.layout = 'unspecified';
end
A.file   = jsonPath;
end
