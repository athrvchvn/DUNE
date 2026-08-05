function d = rootDir()
%ROOTDIR Absolute path of the D:\UWB_3\matlab folder (parent of +dune).
d = fileparts(fileparts(mfilename('fullpath')));
end
