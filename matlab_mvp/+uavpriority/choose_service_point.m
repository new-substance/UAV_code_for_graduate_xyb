function choice=choose_service_point(nav,stage,fit,observed_xy,attempts,cfg,rule)
%CHOOSE_SERVICE_POINT P01 finite position/stage feedback, proposal only.
% Observed means qualified spatial records, not every planned route vertex.
% No future image/radio or physical terminal coordinates are accepted here.
% Actual navigation, distinct-position and return admission follow execution.
if nargin<7,rule='stage';end
assert(ismember(stage,{'LOCALIZE','VERIFY'}) && ismember(rule,{'stage','cost'}));
points=[cfg.scene.reference_centers_xy_m;uavpriority.inset_candidates(cfg)];
n=size(points,1);nr=size(cfg.scene.reference_centers_xy_m,1);
assert(isequal(size(attempts),[n,1]) && all(attempts>=0 & attempts==floor(attempts)));
assert(isempty(observed_xy) || size(observed_xy,2)==2);
choice=struct('available',false,'candidate_id',NaN,'reference_id',NaN,'xy_m',[], ...
    'stage',stage,'rule',rule,'estimate_xy_m',[],'eligible_count',0,'candidates',[]);
if isempty(fit) || any(~isfinite(fit.x)) || ~isfinite(nav.last_reference_id),return;end
x=fit.x(:);assert(numel(x)==2);choice.estimate_xy_m=x;
radius=0;if isfield(fit,'radius') && isfinite(fit.radius),radius=max(0,fit.radius);end
H=jacobian(observed_xy,x,cfg);base=lambda_min(H'*H);best=[Inf,Inf,Inf,Inf];
% Columns: ID, reference, x, y, eligible, attempts, slant, cost, risk, gain.
table=zeros(n,10);
for k=1:n
    ref=mod(k-1,nr)+1;q=points(k,:);eligible=true;
    if attempts(k)>=cfg.stage.navigation_recovery_cap,eligible=false;end
    if ~isempty(observed_xy) && min(vecnorm(observed_xy-q,2,2))< ...
            cfg.localization.distinct_horizontal_separation_m+1,eligible=false;end
    d=norm([q';cfg.scene.flight_height_m]-[x;cfg.scene.terminal_height_m]);
    if d>cfg.scene.design_service_slant_m,eligible=false;end
    path=uavmvp.reference_path(cfg,nav.last_reference_id,ref,[]);
    if ~path.available,eligible=false;end
    offset=norm(q-cfg.scene.reference_centers_xy_m(ref,:));
    cost=path.time_proxy_s+2*offset/cfg.motion_and_budget.horizontal_max_speed_mps+.05*d;
    % Declared radius is a nominal quality quantity, not a guaranteed bound.
    risk=max(0,d+radius-cfg.scene.design_service_slant_m);
    h=jacobian(q,x,cfg);gain=max(0,lambda_min(H'*H+h'*h)-base)/(1+cost);
    table(k,:)=[k,ref,q,eligible,attempts(k),d,cost,risk,gain];
    if ~eligible,continue;end
    choice.eligible_count=choice.eligible_count+1;
    score=[0,0,cost,k];
    if strcmp(rule,'stage')
        if strcmp(stage,'VERIFY'),score=[risk,0,cost,k];
        else,score=[risk,-gain,cost,k];end
    end
    delta=find(abs(score-best)>1e-12,1);
    if ~choice.available || (~isempty(delta) && score(delta)<best(delta))
        best=score;choice.available=true;choice.candidate_id=k;
        choice.reference_id=ref;choice.xy_m=q;
    end
end
choice.candidates=table;
end
function H=jacobian(points,x,cfg)
if isempty(points),H=zeros(0,2);return;end
delta=x'-points;z=cfg.scene.flight_height_m-cfg.scene.terminal_height_m;
H=delta./sqrt(sum(delta.^2,2)+z^2);
end
function value=lambda_min(I)
value=max(0,min(eig((I+I')/2)));
end
