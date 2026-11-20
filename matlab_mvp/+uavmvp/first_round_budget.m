function budget=first_round_budget(cfg,targets)
%FIRST_ROUND_BUDGET Deterministic reservation before unknown radio outcomes.
% All first contacts and all potentially responding members' cold RTTs count.
if isempty(targets),budget=0;return;end
timing=uavmvp.radio_timing(cfg);budget=cfg.motion_and_budget.settle_s;
for group=unique(ceil(targets/cfg.css.group_size))
    n=sum(ceil(targets/cfg.css.group_size)==group);g=uavmvp.radio_timing(cfg,n);
    budget=budget+g.group_s+cfg.css.wake_ready_s+n*(timing.frames.identity+cfg.css.guard_s);
end
budget=budget+numel(targets)*(timing.cold_max_s+cfg.motion_and_budget.calculation_model_s_per_decision);
end
