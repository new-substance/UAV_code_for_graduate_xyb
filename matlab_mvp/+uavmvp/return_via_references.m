function status=return_via_references(world,cfg,deadline)
%RETURN_VIA_REFERENCES Recheck each actual leg; bounded alternate acquisitions.
% Failure is explicit: inability to reacquire a reference is not safe return.
status=struct('arrived',false,'reason','NO_FEASIBLE_REFERENCE_RETURN','steps',{{}}, ...
    'landing',struct('command_completed',false,'reason','RETURN_NOT_REACHED'));
xy=cfg.scene.reference_centers_xy_m;n=size(xy,1);
[~,home]=min(vecnorm(xy-cfg.scene.home_xyz_m(1:2)',2,2));
blocked=[];failures=0;
for stage=1:n+cfg.motion_and_budget.reference_failure_return_replans
    % Recheck age after every failed movement/recovery, not only on entry.
    last_acquired=uavmvp.recent_reference_id(world.nav,cfg);
    [~,order]=sort(vecnorm(xy-world.nav.p(1:2)',2,2));choices=struct([]);
    for id=order(1:min(3,n))'
        if ismember(id,blocked),continue;end
        plan=uavmvp.reference_return_plan(world.nav,cfg,id,blocked);
        if plan.available
            choices=[choices;struct('id',id,'reserve_s',plan.reserve_s,'ids',plan.reference_ids)]; %#ok<AGROW>
        end
    end
    if isempty(choices),return;end
    [~,best]=min([choices.reserve_s]);choice=choices(best);id=choice.id;
    if id==last_acquired && numel(choice.ids)>1,id=choice.ids(2);end
    [~,from_id]=min(vecnorm(xy-world.nav.p(1:2)',2,2));
    step=uavmvp.transit_reference(world,from_id,id,cfg,deadline);step.from_reference_id=from_id;
    response=uavmvp.return_failure_response(step,from_id);step.failure_response=response;
    status.steps{end+1}=step;
    if ~step.accepted
        status.reason=step.reason;failures=failures+1;
        if failures>cfg.motion_and_budget.reference_failure_return_replans,return;end
        if response.stop,return;end
        if response.block_target,blocked=unique([blocked,id]);end
        if response.repair_departure
            % Departure images/approach failed before the target was observed.
            % Actually reacquire the departure marker, within the same finite
            % replan/deadline limits, before retrying a route through the target.
            repair=uavmvp.visit_reference(world,from_id,cfg,deadline);
            repair.kind='RETURN_DEPARTURE_REPAIR';status.steps{end+1}=repair;
            if ~repair.accepted && strcmp(repair.reason,'REFERENCE_BUDGET')
                status.reason=repair.reason;return;
            elseif ~repair.accepted && repair.arrived && ~isempty(repair.attempts)
                blocked=unique([blocked,from_id]);
            end
        end
        continue;
    end
    if id==home
        status.arrived=true;status.reason='HOME_REFERENCE_ACQUIRED';
        status.landing=uavmvp.land_at_home(world,cfg,deadline);return;
    end
end
status.reason='RETURN_STEP_LIMIT';
end
