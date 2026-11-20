function status=repair_departure_reference(w,id,cfg,deadline)
%REPAIR_DEPARTURE_REFERENCE N01/T01 bounded actual departure imagery.
% A rejected long capture forecast must not suppress an existing short,
% observed recovery. No posterior is credited until an actual image passes.
status=struct('accepted',false,'reason','DEPARTURE_REFERENCE_UNRECOVERED', ...
    'onsite_accepted',false,'recovery_attempts',0,'recovery_accepted',false, ...
    'start_s',w.t,'end_s',w.t,'return_reserve_s',NaN);
back=uavmvp.reference_return_plan(w.nav,cfg,id);
frames=cfg.navigation_candidate.reference_max_attempts_per_visit*cfg.navigation_candidate.reference_observation_s;
status.return_reserve_s=back.reserve_s;
if ~isfinite(back.reserve_s) || w.t+frames+back.reserve_s>deadline
    status.reason='DEPARTURE_REFERENCE_RETURN_BUDGET';return;
end
onsite=uavpriority.observe_onsite(w,id,cfg,deadline-back.reserve_s);
status.onsite_accepted=onsite.accepted;
plan=uavmvp.departure_repair_plan(w.nav,id,cfg,deadline);
if ~plan.accepted && strcmp(plan.reason,'DEPARTURE_REPAIR_NAVIGATION_REJECTED')
    back=uavmvp.reference_return_plan(w.nav,cfg,id);status.return_reserve_s=back.reserve_s;
    if isfinite(back.reserve_s) && w.t+frames+back.reserve_s<=deadline
        recovery=uavmvp.recover_reference(w,id,cfg,deadline-back.reserve_s);
        status.recovery_attempts=numel(recovery.attempts);status.recovery_accepted=recovery.accepted;
        plan=uavmvp.departure_repair_plan(w.nav,id,cfg,deadline);
    end
end
if plan.accepted
    step=uavmvp.visit_reference(w,id,cfg,plan.observation_deadline_s);
    back=uavmvp.reference_return_plan(w.nav,cfg,id);
    status.accepted=step.accepted && back.available && w.t+back.reserve_s<=deadline;
    if status.accepted,status.reason='ACTUAL_DEPARTURE_REFERENCE_REPAIRED';end
elseif contains(plan.reason,'BUDGET')
    status.reason='DEPARTURE_REFERENCE_RETURN_BUDGET';
end
status.end_s=w.t;
end
