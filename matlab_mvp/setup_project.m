function setup_project()
% One source root only. Never recursively add the repository or archive.
root=fileparts(mfilename('fullpath'));
addpath(root);
end
