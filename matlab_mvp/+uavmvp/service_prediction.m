function result=service_prediction(tracker,book,histories,beliefs,nav,cfg)
%SERVICE_PREDICTION Conditional geometry/information from online evidence only.
count=numel(beliefs);result=struct('probability',zeros(1,count),'information',zeros(1,count), ...
    'radius_reduction',zeros(1,count),'eligible',false(1,count),'mode',{cell(1,count)});
for id=1:count
    if isempty(beliefs{id}) || ~uavmvp.can_add_position(tracker,book,id,nav,cfg),continue;end
    result.eligible(id)=true;b=beliefs{id};ids=tracker.terminals(id).pool_ids;
    records=book.nodes(id).range_records;old=[];radio_ids=[];
    if ~isempty(ids)
        selected=[records{ids}];old=[selected.effective_nav_snapshot_id];radio_ids=[selected.radio_history_id];
    end
    [preview,new]=uavmvp.nav_clone(nav,cfg);[anchors,P]=uavmvp.nav_archive(preview,[old,new]);
    [radio,future,mode]=uavmvp.predict_radio_record(histories{id},nav.t,cfg);
    C=uavmvp.radio_history_covariance(radio,[radio_ids,future]);result.mode{id}=mode;
    distances=sqrt(sum((b.points-anchors(1:2,end)').^2,2)+(anchors(3,end)-cfg.scene.terminal_height_m)^2);
    % Cell integration and navigation uncertainty smooth the hard service edge.
    width=sqrt(cfg.planner.belief_grid_step_m^2/12+max(eig(P(end-2:end,end-2:end))));
    coverage=.5*(1+erf((cfg.scene.design_service_slant_m-distances)/(sqrt(2)*max(width,1e-6))));
    ne=1+strcmp(mode,'cold');pr=cfg.ranging.baseline_stage_probabilities;
    transport=pr.wake_decode*pr.identity_decode_given_control*pr.range_exchange_valid^ne;
    result.probability(id)=sum(b.weights.*coverage)*transport;
    K=size(b.samples,1);
    for k=1:K
        x=b.samples(k,:)';distance=norm([x;cfg.scene.terminal_height_m]-anchors(:,end));
        if distance>cfg.scene.design_service_slant_m,continue;end
        score=uavmvp.conditional_information(x,anchors,P,C,cfg);
        result.information(id)=result.information(id)+score.information_gain*transport/K;
        reduction=max(0,score.radius_before-score.radius_after);
        if strcmp(cfg.planner.geometry_proxy,'four_fit')
            if strcmp(b.phase,'VERIFY')
                % A prospective heldout is not added to the four-fit geometry.
                reduction=0;
            elseif numel(ids)>=4
                quartet=uavmvp.four_fit_geometry(x,anchors,P,C,cfg);
                reduction=quartet.radius_reduction;
            end
        end
        result.radius_reduction(id)=result.radius_reduction(id)+reduction*transport/K;
    end
end
end
