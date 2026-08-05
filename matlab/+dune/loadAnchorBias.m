function B = loadAnchorBias(jsonPath)
%LOADANCHORBIAS Load per-device additive range biases (m) from anchor_bias.json.
%   B.anchorIds, B.anchorBias [same order]; B.tagIds, B.tagBias (NaN if unset).
%   corrected_range = raw - bias_anchor - bias_tag

if nargin < 1 || isempty(jsonPath)
    jsonPath = fullfile(dune.rootDir(), 'config', 'anchor_bias.json');
end
raw = jsondecode(fileread(jsonPath));

n = numel(raw.anchors);
B.anchorIds  = zeros(n,1);
B.anchorBias = zeros(n,1);
for k = 1:n
    a = raw.anchors(k);
    B.anchorIds(k)  = a.id;
    B.anchorBias(k) = a.bias_m;
end

B.tagIds = []; B.tagBias = [];
if isfield(raw, 'tags')
    for k = 1:numel(raw.tags)
        t = raw.tags(k);
        B.tagIds(end+1,1) = t.id;
        if isnumeric(t.bias_m) && ~isempty(t.bias_m)
            B.tagBias(end+1,1) = t.bias_m;
        else
            B.tagBias(end+1,1) = NaN;
        end
    end
end
B.file = jsonPath;
end
