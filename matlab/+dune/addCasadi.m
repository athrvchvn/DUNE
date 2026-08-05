function ok = addCasadi(userPath)
%ADDCASADI Put CasADi on the MATLAB path for the moving-horizon estimators.
%   ok = dune.addCasadi()            % resolve automatically
%   ok = dune.addCasadi(userPath)    % try userPath first
%
%   dune.MheEstimator and dune.MheRigid need CasADi + IPOPT (nlpsol). Every
%   other estimator (dune.FusionEkf, dune.multilaterate) is toolbox-free, so a
%   missing CasADi must degrade to "MHE unavailable", never break the pipeline.
%
%   Resolution order:
%     1. userPath, when given and it exists
%     2. $CASADI_ROOT  -- the per-machine escape hatch; set this and forget it
%     3. CasADi already reachable (casadi.SX resolves)
%     4. common install locations for this OS
%
%   Returns true if CasADi is usable afterwards. Ported from the six DUNE
%   scripts that each hardcoded one contributor's Windows download folder.

if nargin < 1, userPath = ""; end
cands = string.empty;

if strlength(string(userPath)) > 0
    cands(end+1) = string(userPath);
end

env = getenv('CASADI_ROOT');
if ~isempty(env)
    cands(end+1) = string(env);
end

for c = cands
    if isfolder(c)
        addpath(char(c));
        if dune.addCasadi_available(), ok = true; return; end
    end
end

% Already on the path (e.g. installed as a toolbox, or added by startup.m)?
if dune.addCasadi_available(), ok = true; return; end

if ispc
    guesses = ["C:\casadi", "C:\Program Files\casadi", ...
               fullfile(getenv('USERPROFILE'), 'Downloads', 'casadi-3.7.0')];
else
    guesses = ["/opt/casadi", "/usr/local/casadi", ...
               fullfile(getenv('HOME'), 'casadi'), ...
               fullfile(getenv('HOME'), 'Downloads', 'casadi-3.7.0')];
end
for g = guesses
    if isfolder(g)
        addpath(char(g));
        if dune.addCasadi_available(), ok = true; return; end
    end
end

ok = false;
warning('dune:addCasadi:notFound', ...
    ['CasADi not found -- the MHE estimators are unavailable.\n' ...
     'Set the CASADI_ROOT environment variable to your CasADi folder, or ' ...
     'pass casadiPath="...". Download: https://web.casadi.org/get/']);
end
