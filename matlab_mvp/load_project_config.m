function cfg=load_project_config()
% Load the explicit configuration, including its migration/release status.
% Retained legacy numeric inputs do not approve a new civil-model mission.
repo=fileparts(fileparts(mfilename('fullpath')));
cfg=jsondecode(fileread(fullfile(repo,'research_configs','baseline.json')));
assert(cfg.localization.fit_position_count==4 && cfg.localization.heldout_position_count==1);
assert(~cfg.navigation_candidate.unknown_terminal_rtt_updates_navigation);
end
