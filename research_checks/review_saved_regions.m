function report=review_saved_regions(output)
%REVIEW_SAVED_REGIONS Read-only counterfactual on saved four-point inputs.
% New quality rules do NOT retroactively complete these old task outcomes.
repo=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(repo,'matlab_mvp'));
setup_project();c=load_project_config();c.localization.quality_method='grid_conditional';rows={};
for seed=[2026112002,2026112003]
    p=fullfile(repo,'results','current',sprintf('dev_%d_n1_stage',seed),sprintf('events_%d.jsonl',seed));
    lines=splitlines(strtrim(fileread(p)));count=0;
    for k=1:numel(lines)
        e=jsondecode(lines{k});
        if ~strcmp(e.kind,'LOCALIZATION') || isempty(e.data.detail.fit),continue;end
        saved=e.data.detail;count=count+1;b=saved.input;
        % JSON numbers/cells are restored to the production bundle layout.
        b.record_ids=b.record_ids(:)';b.nav_ids=b.nav_ids(:)';
        if ~iscell(b.constituent_ids)
            b.constituent_ids=num2cell(b.constituent_ids,2)';
        end
        b.aggregation_matrix=eye(4);b.model_bias_bounds_m=b.model_bias_bounds_m(:);
        f=uavpriority.fit_position(b,c);
        rows{end+1}=struct('seed',seed,'sequence',count,'time_s',e.time_s, ...
            'old_reason',saved.fit.reason,'old_radius_m',saved.fit.radius, ...
            'new_reason',f.reason,'new_radius_m',f.radius,'quality',f.quality, ...
            'estimate_change_m',norm(f.x-saved.fit.x),'new_mission_success',false); %#ok<AGROW>
    end
end
report=struct('kind','saved_four_point_counterfactual_not_mission_evaluation', ...
    'new_missions',0,'rows',{rows});
assert(~isfile(output),'uavproject:OutputExists','Use a new report output.');
encoded=jsonencode(report);assert(numel(unicode2native(encoded,'UTF-8'))<100000);
uavmvp.write_json(output,report);
end
