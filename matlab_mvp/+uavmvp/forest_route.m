function out=forest_route(action,in,c)
%FOREST_ROUTE P02/M01/N01: finite graph -> declared stop-to-stop nominal model.
% Authoring a provider does not authorize running a trajectory or a task.
switch action
    case 'plan'
        out=plan_route(in,c);
    case 'profile'
        out=make_profile(in,c);
    otherwise,error('forest:Action','Use plan or profile.');
end
end

function out=plan_route(in,c)
n=in.navigation;map=in.map;target=in.target_vertex;
assert(isscalar(target) && target==fix(target) && target>=1 && target<=size(map.points_m,1));
assert(norm(n.v)<=c.motion_model.estimated_arrival_speed_tolerance_m_per_s,'forest:InitialMotion','Stop-to-stop route requires a settled initial estimate.');
assert(~(n.p(3)>c.motion_model.estimated_arrival_position_tolerance_m && ...
    map.points_m(target,3)<=1e-8),'forest:LandingRequired','Ground descent must use the separate CSS-maintained landing phase.');
assert(all(abs(n.p(1:2))<=c.scene.allowed_corridor_half_width_m),'forest:Corridor','Current estimate outside declared corridor.');
[~,start]=min(vecnorm(map.points_m-n.p',2,2));
vertices=shortestpath(map.graph,start,target);assert(~isempty(vertices),'forest:Corridor','No connected candidate path.');
points=map.points_m(vertices,:);points=[n.p';points];
keep=[true;vecnorm(diff(points),2,2)>1e-10];points=points(keep,:);
assert(all(all(abs(points(:,1:2))<=c.scene.allowed_corridor_half_width_m)));
count=max(0,size(points,1)-1);seconds=zeros(count,1);
for j=1:count
    d=points(j+1,:)-points(j,:);L=norm(d);direction=d/L;
    horizontal=norm(direction(1:2));vertical=abs(direction(3));
    vmax=Inf;amax=Inf;
    if horizontal>1e-12
        vmax=min(vmax,c.service.cruise_speed_m_per_s/horizontal);
        amax=min(amax,c.service.acceleration_m_per_s2/horizontal);
    end
    if vertical>1e-12
        vmax=min(vmax,c.motion_model.vertical_cruise_speed_limit_m_per_s/vertical);
        amax=min(amax,c.motion_model.vertical_acceleration_limit_m_per_s2/vertical);
    end
    if L<=vmax^2/amax,t=2*sqrt(L/amax);else,t=L/vmax+vmax/amax;end
    seconds(j)=t+c.service.movement_tracking_and_settling_allowance_s;
end
out=struct('start_time_s',n.t,'version',n.version,'start_position_m',n.p, ...
    'waypoints_m',points(2:end,:),'leg_reservation_s',seconds, ...
    'move_and_settle_s',sum(seconds),'target_vertex',target,'target_m',map.points_m(target,:), ...
    'scope','nominal_command_and_tracking_budget_not_actual_arrival_guarantee');
end

function out=make_profile(in,c)
n=in.navigation;p=in.plan;dt=c.initialization.imu_sample_interval_s;
assert(p.version==n.version && p.start_time_s==n.t && norm(p.start_position_m-n.p)<1e-10,'forest:Snapshot','Stale route plan.');
assert(in.hold_s>0 && isfinite(in.hold_s));
total=p.move_and_settle_s+in.hold_s;
assert(total<=c.service.reference_max_age_s && n.t+total<=c.service.task_resource_deadline_s,'forest:ForecastBudget','Complete route/hold exceeds registered horizon.');
assert(isfinite(n.last_reference_s) && n.t+total<=n.last_reference_s+c.service.reference_max_age_s, ...
    'forest:ForecastBudget','Ordinary route requires an actual unexpired reference service.');
assert(size(p.waypoints_m,1)<=c.algorithm.max_candidate_actions,'forest:ForecastBudget','Route waypoint limit.');
clock=tic;enforce=true;
if isfield(in,'execution_kind'),assert(strcmp(in.execution_kind,'fixed_input_review'));enforce=false;end
limit=ceil(c.service.reference_max_age_s/dt)+c.algorithm.max_candidate_actions+1;
profile=repmat(struct('R',eye(3),'specific_force_body_mps2',zeros(3,1),'body_rate_radps',zeros(3,1),'duration_s',0),limit,1);
w=in.body_rate_radps(:);count=0;shadow=n;valid=true;reason='complete_nominal_profile';
targets=[p.waypoints_m;p.target_m];durations=[p.leg_reservation_s;in.hold_s];
for j=1:numel(durations)
    remaining=durations(j);
    while remaining>1e-10
        if enforce && toc(clock)>c.service.computation_bound_s,error('forest:ComputationTimeout','Waypoint nominal-profile budget exceeded.');end
        count=count+1;assert(count<=limit,'forest:ForecastBudget','Nominal-profile sample limit.');
        step=min(dt,remaining);
        a=struct('navigation',shadow,'setpoint_m',targets(j,:),'landing',false,'duration_s',step,'body_rate_radps',w);
        result=uavmvp.forest_motion('nominal_step',a,c);shadow=result.navigation;w=result.body_rate_radps;
        profile(count)=result.profile;remaining=remaining-step;
        if any(abs(shadow.p(1:2))>c.scene.allowed_corridor_half_width_m)
            valid=false;reason='nominal_corridor_exit';break
        end
    end
    arrived=norm(shadow.p-targets(j,:)')<=c.motion_model.estimated_arrival_position_tolerance_m && ...
        norm(shadow.v)<=c.motion_model.estimated_arrival_speed_tolerance_m_per_s;
    if ~valid || ~arrived,valid=false;if arrived,reason='nominal_corridor_exit';else,reason='nominal_tracking_budget_exceeded';end;break,end
end
out=struct('valid',valid,'reason',reason,'profile',profile(1:count),'mean_belief',shadow, ...
    'version',n.version,'start_time_s',n.t,'forecast_covariance_applied',false);
if ~valid,return,end
input=struct('navigation',n,'profile',profile(1:count));
if ~enforce,input.execution_kind='fixed_input_review';end
forecast=uavmvp.forest_forecast('profile',input,c);
if enforce && toc(clock)>c.service.computation_bound_s,error('forest:ComputationTimeout','Combined profile/covariance budget exceeded.');end
out.mean_belief.P=forecast.predicted_covariance;out.forecast=forecast;
out.forecast_covariance_applied=true;out.valid=forecast.navigation_ok;
if ~out.valid,out.reason='nominal_navigation_cap_exceeded';end
end
