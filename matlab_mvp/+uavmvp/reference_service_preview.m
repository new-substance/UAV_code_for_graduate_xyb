function preview=reference_service_preview(arrival,acquire,window_s,start_s,reference_id,tracker,book,histories,beliefs,cfg,deadline)
%REFERENCE_SERVICE_PREVIEW Common conditional first-service model for ranking.
% Does not add an action, provide pixels, modify actual navigation or authorize
% service. Costs are conditional/worst-window proxies, not expected mission time.
attempts=cfg.navigation_candidate.reference_max_attempts_per_visit;
onsite=attempts*cfg.navigation_candidate.reference_observation_s;
first=arrival;qA=1;
if acquire
    acquisition=uavmvp.reference_belief_preview(first,reference_id,cfg,attempts);
    qA=acquisition.success_probability_proxy;first=acquisition.conditional_nav;
end
decision=uavmvp.predict_hold(first,cfg.motion_and_budget.calculation_model_s_per_decision,cfg);
first=decision.nav;
value=uavmvp.service_prediction(tracker,book,histories,beliefs,first,cfg);
single=uavmvp.predict_service_visit(first,first.p,reference_id,cfg,window_s,onsite);
back=uavmvp.reference_return_plan(single.reacquisition.nav,cfg,reference_id);
duration=first.t-start_s+single.duration_s;
admissible=qA>0 && single.accepted && back.available && start_s+duration+back.reserve_s<=deadline;
weight=qA*double(admissible);known=[book.nodes.discovered];
probability=zeros(size(known));information=probability;reduction=probability;
if weight>0
    probability=weight*value.probability.*known;
    information=weight*value.information.*known;
    reduction=weight*value.radius_reduction.*known;
end
summary=struct('model','conditional_reference','reference_weight',qA,'admissible',admissible, ...
    'service_epoch_s',first.t,'duration_proxy_s',duration,'return_reserve_s',back.reserve_s, ...
    'probability',probability,'information',information,'radius_reduction',reduction,'eligible',value.eligible);
preview=struct('nav',first,'value',value,'summary',summary);
end
