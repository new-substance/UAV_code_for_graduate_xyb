function [score,tail]=stage_candidate_score(need,Q,durations,positions,chosen,reduction,reserve,cfg,followup_cap)
%STAGE_CANDIDATE_SCORE Full stage score and three controlled scoring ablations.
% No ablation changes eligibility, physical service, quality gates or return.
mode=cfg.planner.scoring_ablation;
assert(ismember(mode,{'none','no_stage','no_sharing','local_cost'}));
stage_need=need;
if strcmp(mode,'no_stage'),stage_need=double(need>0);end
improvement=min(stage_need,reduction/cfg.localization.nominal_region_radius_limit_m);
if strcmp(mode,'local_cost')
    tail=[];
    score=durations(chosen)-median(durations)*sum(improvement);
elseif strcmp(mode,'no_sharing')
    % Focus one terminal in the score. Other terminals retain their independent
    % unmet-work penalty; co-service remains available in the actual executor.
    active=find(need>0);best=max(Q,[],1);
    baseline=need./max(best,.05)*max(durations);
    values=inf(size(active));tails=cell(size(active));
    for j=1:numel(active)
        id=active(j);
        tails{j}=uavmvp.shared_tail_cost(need(id),Q(:,id),durations,positions,chosen,cfg,followup_cap);
        values(j)=durations(chosen)+tails{j}.time_proxy_s+sum(baseline)-baseline(id)+reserve- ...
            median(durations)*improvement(id);
    end
    if isempty(active)
        score=durations(chosen)+reserve;tail=[];return;
    end
    [score,j]=min(values);id=active(j);tail=tails{j};
    tail.time_proxy_s=tail.time_proxy_s+sum(baseline)-baseline(id);
    tail.unresolved_penalty_s=tail.unresolved_penalty_s+sum(baseline)-baseline(id);
    unresolved=need;unresolved(id)=tail.unresolved;tail.unresolved=unresolved;
    tail.focus_terminal=id;
else
    tail=uavmvp.shared_tail_cost(stage_need,Q,durations,positions,chosen,cfg,followup_cap);
    score=durations(chosen)+tail.time_proxy_s+reserve-median(durations)*sum(improvement);
end
end
