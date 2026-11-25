function cfg=load_project_config(mode)
%LOAD_PROJECT_CONFIG Explicit whitelist: online model never sees review fixtures.
% S01/H01. Legacy configuration is available only by explicit historical mode.
if nargin<1,mode='forest';end
repo=fileparts(fileparts(mfilename('fullpath')));
raw=jsondecode(fileread(fullfile(repo,'research_configs','baseline.json')));
if strcmp(mode,'legacy')
    cfg=raw;return
end
assert(strcmp(mode,'forest'),'uavproject:ConfigMode','Use forest or explicit legacy.');
m=raw.project_context.joint_model_candidate;
keys={'scene','radio','imu','service','initialization','motion_model', ...
    'navigation_admission','finite_policy_limits','landing','evaluation','algorithm'};
cfg=struct('model_id',m.id,'kind','forest_css_imu','mission_enabled',false);
for k=1:numel(keys),cfg.(keys{k})=m.(keys{k});end
assert(~isfield(cfg,'project_context') && ~isfield(cfg,'fixed_review'));
assert(cfg.algorithm.fit_group_count==4 && cfg.algorithm.heldout_group_count==1);
assert(~cfg.algorithm.unknown_updates_navigation);
end
