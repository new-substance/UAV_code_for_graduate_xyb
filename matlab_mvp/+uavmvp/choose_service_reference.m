function choice=choose_service_reference(nav,current,visits,tracker,book,histories,cfg,deadline)
%CHOOSE_SERVICE_REFERENCE Nominal geometry proxy for choosing a reference visit.
% Future covariance/visual success is NOT certified by this coarse ranking.
if nargin<8,deadline=cfg.motion_and_budget.time_limit_s;end
beliefs=uavmvp.planning_belief(tracker,book,histories,nav,cfg);xy=cfg.scene.reference_centers_xy_m;
rows=struct([]);
for id=1:size(xy,1)
    if visits(id)>=cfg.scene.max_sweeps,continue;end
    path=uavmvp.reference_path(cfg,current,id);if ~path.available,continue;end
    candidates=uavmvp.service_candidate_points(id,cfg);points=candidates(:,1:2);
    best=0;
    for j=1:size(points,1)
        p=points(j,:);if any(p<0) || p(1)>cfg.scene.width_m || p(2)>cfg.scene.height_m,continue;end
        value=0;
        for terminal=1:numel(beliefs)
            b=beliefs{terminal};if isempty(b),continue;end
            if ~isempty(b.bundle) && any(vecnorm(b.bundle.anchors(1:2,:)-p')<cfg.localization.distinct_horizontal_separation_m+.2),continue;end
            d=sqrt(sum((b.points-p).^2,2)+cfg.scene.flight_height_m^2);
            probability=sum(b.weights(d<=cfg.scene.design_service_slant_m));
            phase=1+strcmp(b.phase,'VERIFY');geometry=0;
            if ~isempty(b.bundle)
                a=b.bundle.anchors;diff=a(1:2,:)-b.mean;norms=sqrt(sum(diff.^2,1)+(a(3,:)-cfg.scene.terminal_height_m).^2);
                H=(diff./norms)';I=H'*H;new=(p'-b.mean)/sqrt(sum((p'-b.mean).^2)+cfg.scene.flight_height_m^2);
                geometry=max(0,min(eig(I+new*new'))-min(eig(I)));
            end
            age=nav.t-book.nodes(terminal).last_contact_s;
            waiting=1+min(1,max(0,age/cfg.planner.terminal_max_wait_s));
            value=value+probability*(phase+geometry)*waiting;
        end
        best=max(best,value);
    end
    if best<=.02,continue;end
    rows=[rows;struct('reference_id',id,'value',best,'path',path, ...
        'score',best/(path.time_proxy_s+cfg.motion_and_budget.stop_max_dwell_s+ ...
        cfg.navigation_candidate.reference_observation_s*cfg.navigation_candidate.reference_max_attempts_per_visit))]; %#ok<AGROW>
end
choice=uavmvp.screen_reference_choices(nav,current,rows,cfg,deadline);
end
