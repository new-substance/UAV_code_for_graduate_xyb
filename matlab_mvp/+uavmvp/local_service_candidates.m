function choice=local_service_candidates(nav,reference_id,tracker,book,histories,cfg,deadline,policy,remaining_local_actions)
%LOCAL_SERVICE_CANDIDATES Common local action set with strict first-leg checks.
% Off-center actions first acquire a real reference image; serving is then
% checked from that posterior. Conditional covariance is optional for ranking
% only; every policy retains the same admission checks and actual executor.
if nargin<8,policy='stage_shared';end
if nargin<9,remaining_local_actions=1;end
assert(isscalar(remaining_local_actions) && remaining_local_actions>=1 && ...
    remaining_local_actions==floor(remaining_local_actions));
points=uavmvp.service_candidate_points(reference_id,cfg);
onsite_s=cfg.navigation_candidate.reference_max_attempts_per_visit*cfg.navigation_candidate.reference_observation_s;
beliefs=uavmvp.planning_belief(tracker,book,histories,nav,cfg);
need=zeros(1,numel(beliefs));known=[book.nodes.discovered];
for id=find(known),if ~isempty(beliefs{id}),need(id)=beliefs{id}.need;end,end
rows=struct([]);plans={};forecast_navs={};forecast_values={};
assert(ismember(cfg.planner.prediction_model,{'arrival','conditional_reference'}));
for k=1:size(points,1)
    target=points(k,:)';conditional=norm(target-nav.p)>.4;
    if conditional
        prep=uavmvp.predict_service_visit(nav,target,reference_id,cfg, ...
            cfg.navigation_candidate.reference_max_attempts_per_visit*cfg.navigation_candidate.reference_observation_s);
        frame=uavmvp.predict_hold(prep.outward.nav,cfg.navigation_candidate.reference_observation_s,cfg);
        margin=uavmvp.reference_margin(frame.nav,reference_id,cfg);
        if ~prep.accepted || ~margin.envelope_inside,continue;end
        predicted=prep.outward.nav;
    else
        prep=[];predicted=nav;
    end
    value=uavmvp.service_prediction(tracker,book,histories,beliefs,predicted,cfg);
    window=min(cfg.motion_and_budget.stop_max_dwell_s, ...
        uavmvp.first_round_budget(cfg,find(value.eligible))+cfg.planner.optional_service_window_s);
    if conditional
        back=uavmvp.reference_return_plan(prep.reacquisition.nav,cfg,reference_id);
        duration=prep.duration_s+window+onsite_s;
    else
        direct=uavmvp.predict_service_visit(nav,target,reference_id,cfg,window,onsite_s);
        if ~direct.accepted,continue;end
        back=uavmvp.reference_return_plan(direct.reacquisition.nav,cfg,reference_id);duration=direct.duration_s;prep=direct;
    end
    if ~back.available || nav.t+duration+back.reserve_s>deadline,continue;end
    q=value.probability;q(~known)=0;
    if sum(q)<.02,continue;end
    rows=[rows;struct('target',target,'conditional_on_actual_reference',conditional, ...
        'window_s',window,'duration_proxy_s',duration,'return_reserve_s',back.reserve_s, ...
        'probability',q,'information',value.information.*known,'radius_reduction',value.radius_reduction.*known, ...
        'score',NaN,'tail',[],'prediction',[])]; %#ok<AGROW>
    plans{end+1}=prep; %#ok<AGROW>
    forecast_navs{end+1}=predicted; %#ok<AGROW>
    forecast_values{end+1}=value; %#ok<AGROW>
end
choice=struct('accepted',false,'selected',[],'plan',[],'candidates',rows,'needs',need,'policy',policy, ...
    'remaining_local_actions',remaining_local_actions,'scoring_ablation',cfg.planner.scoring_ablation);
if isempty(rows),return;end
% Share the same first-action forecast across stage, information and horizon
% policies. The legacy lookahead_reference alias still requests that forecast
% explicitly, preserving historical commands; new comparisons use the common
% configuration switch instead of giving one policy extra prediction ability.
conditional=strcmp(cfg.planner.prediction_model,'conditional_reference') || strcmp(policy,'lookahead_reference');
previews=cell(1,numel(rows));
for j=1:numel(rows)
    if conditional
        previews{j}=uavmvp.reference_service_preview(forecast_navs{j},rows(j).conditional_on_actual_reference, ...
            rows(j).window_s,nav.t,reference_id,tracker,book,histories,beliefs,cfg,deadline);
        rows(j).prediction=previews{j}.summary;
    else
        rows(j).prediction=struct('model','arrival','reference_weight',1,'admissible',true, ...
            'service_epoch_s',forecast_navs{j}.t,'duration_proxy_s',rows(j).duration_proxy_s, ...
            'return_reserve_s',rows(j).return_reserve_s,'probability',rows(j).probability, ...
            'information',rows(j).information,'radius_reduction',rows(j).radius_reduction, ...
            'eligible',forecast_values{j}.eligible);
    end
end
predictions=[rows.prediction];Q=vertcat(predictions.probability);
durations=[predictions.duration_proxy_s]';xy=reshape([rows.target],3,[])';
for j=1:numel(rows)
    switch policy
        case 'stage_shared'
            % Geometry gap represents the extra fit work that point count alone
            % misses. It is a bounded cost correction, not an acceptance test.
            [rows(j).score,rows(j).tail]=uavmvp.stage_candidate_score(need,Q,durations,xy(:,1:2),j, ...
                rows(j).prediction.radius_reduction,rows(j).prediction.return_reserve_s,cfg,remaining_local_actions-1);
        case {'information','lookahead','lookahead_reference'}
            rows(j).score=-sum(rows(j).prediction.information)/max(rows(j).prediction.duration_proxy_s,1e-6);
        case {'fixed','fixed_coverage'}
            rows(j).score=j;
        otherwise,error('uavmvp:plannerPolicy','Unknown common-set policy.');
    end
end
if strcmp(policy,'lookahead') && ~conditional
    rows=uavmvp.information_lookahead(rows,forecast_navs,nav,reference_id,tracker,book,histories,beliefs,cfg,deadline,remaining_local_actions);
elseif ismember(policy,{'lookahead','lookahead_reference'})
    rows=uavmvp.reference_information_lookahead(rows,forecast_navs,nav,reference_id,tracker,book,histories,beliefs,cfg,deadline,remaining_local_actions,previews);
end
[~,best]=min([rows.score]);choice.accepted=true;choice.selected=rows(best);
choice.plan=plans{best};choice.candidates=rows;
end
