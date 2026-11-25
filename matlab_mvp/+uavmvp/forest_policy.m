function out=forest_policy(action,in,c)
%FOREST_POLICY I02/P01/P02/F03: finite graph and same-capability B0/B1 selection.
switch action
    case 'initial'
        step=c.finite_policy_limits.service_candidate_grid_step_m;h=c.scene.allowed_corridor_half_width_m;
        axis=-h:step:h;[x,y]=ndgrid(axis,axis);xy=[x(:),y(:)];
        q=[xy,repmat(c.scene.nominal_imu_cruise_z_m,size(xy,1),1)];
        q(end+1,:)=c.scene.home_imu_xyz_m(:)';home=size(q,1);
        D=square_distances(q);[i,j]=find(triu(abs(D-step)<1e-8,1));
        [~,above]=min(vecnorm(q(1:end-1,1:2)-q(home,1:2),2,2));
        i=[i;above];j=[j;home];w=[D(sub2ind(size(D),i(1:end-1),j(1:end-1)));norm(q(above,:)-q(home,:))];
        G=graph(i,j,w,size(q,1));
        half=c.scene.unknown_deployment_half_width_m;d=c.finite_policy_limits.coverage_grid_step_m;
        cells=[];yy=-half:d:half;
        for k=1:numel(yy),xx=-half:d:half;if mod(k,2)==0,xx=fliplr(xx);end;cells=[cells;xx(:),repmat(yy(k),numel(xx),1)];end %#ok<AGROW>
        order=zeros(size(cells,1),1);
        for k=1:numel(order),[~,order(k)]=min(vecnorm(q(:,1:2)-cells(k,:),2,2)+abs(q(:,3)-c.scene.nominal_imu_cruise_z_m));end
        route=home;last=home;
        for next=[order(:)' home]
            path=shortestpath(G,last,next);assert(~isempty(path),'forest:Corridor','Disconnected coverage graph.');
            route=[route path(2:end)];last=next; %#ok<AGROW>
        end
        out=struct('points_m',q,'graph',G,'coverage_vertices',order,'route_vertices',route,'home_vertex',home);
    case 'segment'
        L=in;assert(isfinite(L) && L>=0);v=c.service.cruise_speed_m_per_s;a=c.service.acceleration_m_per_s2;
        if L<=v*v/a,out=2*sqrt(L/a);else,out=L/v+v/a;end
    case 'information'
        CF=in.Cff;B=in.Cqf;S=in.Cqq-B*(CF\B');A=in.Aq-B*(CF\in.Af);
        L=chol((S+S')/2,'lower');W=L\A;out=W'*W;
    case 'select'
        out=select_action(in,c);
    otherwise,error('forest:Action','Unknown policy action.');
end
end

function out=select_action(in,c)
assert(ismember(in.rule,{'B0','B1'}));s=in.state;now=in.now_s;
assert(now>=0 && in.return_reservation_s>=0 && in.next_reference_reservation_s>0);
assert(numel(in.actions)<=c.algorithm.max_candidate_actions,'forest:CandidateBudget','Too many finite actions.');
if isempty(in.actions)
    out=struct('kind','RETURN_OR_TERMINATE','reason','empty_finite_action_set','selected',[]);return
end
assert(numel(unique([in.actions.id]))==numel(in.actions),'forest:CandidateID','Duplicate candidate IDs.');
if s.no_progress>=c.finite_policy_limits.max_consecutive_no_progress_decisions
    out=struct('kind','RETURN_OR_TERMINATE','reason','no_progress_limit','selected',[]);return
end
feasible=[];keys=[];
for k=1:numel(in.actions)
    a=in.actions(k);assert(a.id>=0 && a.move_and_settle_s>=0 && a.shared_control_s>=0 && ...
        all(isfinite([a.id,a.move_and_settle_s,a.shared_control_s,a.geometry_gain,a.fixed_order])));
    total=a.move_and_settle_s+a.shared_control_s+c.service.computation_bound_s;
    counts=zeros(1,5);phases={'NOTICE','VERIFY','DATA','INITIAL_FIT','IMPROVE'};
    % Opportunities are computed from legal regions/clone geometry by the caller;
    % no hidden-node location or future packet outcome is an interface field.
    selected=strings(0,1);
    opp=a.opportunities;
    if ~isempty(opp)
        pri=zeros(numel(opp),1);
        for j=1:numel(opp)
            p=find(strcmp(phases,opp(j).phase),1);
            if isempty(p),p=numel(phases)+1;end
            pri(j)=p;
        end
        % Same phase/ID packing for B0 and B1; only complete-action choice differs.
        labels=reshape(string({opp.id}),[],1);[~,idOrder]=sort(labels);rank=zeros(size(idOrder));rank(idOrder)=1:numel(idOrder);
        [~,order]=sortrows([pri,rank],[1 2]);opp=opp(order);
    end
    for j=1:numel(opp)
        o=opp(j);idx=find([s.nodes.id]==string(o.id));assert(isscalar(idx));n=s.nodes(idx);
        phase=uavmvp.forest_protocol('phase',n,c);
        assert(strcmp(phase,o.phase),'forest:Phase','Stale opportunity phase.');
        if ~o.eligible || strcmp(phase,'NONE') || strcmp(phase,'UNSEEN'),continue,end
        assert(~ismember(string(o.id),selected),'forest:Duplicate','Repeated node opportunity.');
        job=struct('phase',phase,'payload_bytes',o.payload_bytes);
        if isfield(o,'missing_pairs'),job.missing_pairs=o.missing_pairs;end
        cost=uavmvp.forest_protocol('cost',job,c);
        proposed=total+cost.seconds;
        if admitted(a,proposed,in,c)
            selected(end+1)=string(o.id);total=proposed;counts(strcmp(phases,phase))=counts(strcmp(phases,phase))+1; %#ok<AGROW>
        end
    end
    hasProgress=any(counts) || a.coverage_progress || a.reference;
    protect=~in.coverage_protection || ~a.insertion || ...
        (a.adjacent_to_next_coverage && s.insertions_on_edge<c.finite_policy_limits.max_insertions_per_coverage_edge);
    if ~hasProgress || ~protect || ~admitted(a,total,in,c),continue,end
    if a.reference
        if s.reference_recoveries>=c.finite_policy_limits.reference_recovery_blocks,continue,end
        total=total+a.reference_service_s;
        if ~admitted(a,total,in,c),continue,end
    end
    if a.insertion && s.extra_waypoints>=c.finite_policy_limits.max_active_extra_waypoints,continue,end
    b=a;b.total_reservation_s=total;b.selected_ids=selected;b.phase_counts=counts;
    if isempty(feasible),feasible=b;else,feasible(end+1)=b;end %#ok<AGROW>
    due=now+total+in.next_reference_reservation_s>=in.reference_deadline_s;
    urgent=double(a.reference && (in.reference_due || due));
    wake=double(a.coverage_progress && in.wake_due && a.has_wake_ticket);
    if strcmp(in.rule,'B0'),key=[urgent,wake,-a.fixed_order,zeros(1,5),-total,-a.id];
    else,key=[urgent,wake,counts,max(0,a.geometry_gain),-total,-a.id];end
    keys(end+1,:)=key; %#ok<AGROW>
end
if isempty(feasible)
    out=struct('kind','RETURN_OR_TERMINATE','reason','no_admissible_complete_action','selected',[]);return
end
[~,order]=sortrows(keys,-(1:size(keys,2)));pick=feasible(order(1));
if isempty(pick.path_m),next=pick.target_m;else,next=pick.path_m(1,:);end
out=struct('kind','ACTION','reason','finite_lexicographic_choice','selected',pick,'next_waypoint_m',next);
end

function ok=admitted(a,total,in,c)
common=a.corridor_valid && a.model_valid && a.prediction_valid && ...
    in.now_s+total+in.return_reservation_s<=c.service.task_resource_deadline_s;
if a.reference
    ok=common && a.recovery_domain_valid; % RF recovery may improve a failed ordinary nav gate.
else
    ok=common && a.navigation_ratio_upper<=1 && ...
        in.now_s+total+in.next_reference_reservation_s<=in.reference_deadline_s;
end
end

function D=square_distances(q)
N=size(q,1);D=zeros(N);
for k=1:N,D(k,:)=vecnorm(q-q(k,:),2,2)';end
end
