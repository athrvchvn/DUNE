function tf = addCasadi_available()
%ADDCASADI_AVAILABLE True when CasADi is importable from the current path.
%   Helper for dune.addCasadi. Probes the class rather than `exist('casadi')`
%   because the package folder can be on the path while the MEX/library part is
%   broken -- constructing an SX is what actually proves it works.
try
    casadi.SX.sym('probe');
    tf = true;
catch
    tf = false;
end
end
