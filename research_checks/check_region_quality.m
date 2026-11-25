function out=check_region_quality()
%CHECK_REGION_QUALITY Q01/V01 counterexamples, never a flight or Monte Carlo.
repo=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(repo,'matlab_mvp'));
setup_project();c=load_project_config('legacy');c.localization.quality_method='grid_conditional';
a=[10,10,15;50,10,15;10,50,15;50,50,15;30,10,15]';x=[30;30];
b=make_bundle(a,x);v=sin((1:15)');b.Pnav=b.Pnav+.005*(v*v');
b.Pnavigation=b.Pnav;four=uavpriority.subbundle(b,1:4);
points=[x';20,20;40,35];
[lo,hi,nominal]=uavpriority.likelihood_bounds(points,four,0);
assert(max(abs(lo-nominal))<1e-12 && max(abs(hi-nominal))<1e-12);
four.model_bias_bounds_m=[.03;.01;.04;.02];
[lo,hi]=uavpriority.likelihood_bounds(points,four,0);
% Box corners plus an interior point verify the bound against a direct solve.
delta=[2*(dec2bin(0:15,4)-'0')-1; .2,-.3,.7,-.1].*four.model_bias_bounds_m';
for j=1:size(points,1)
    [d,~,C]=uavmvp.range_model(points(j,:)',four.anchors,four.Pnav,four.Cradio,0);
    L=chol(C,'lower');
    for k=1:size(delta,1)
        e=four.ranges-d-delta(k,:)';value=sum((L\e).^2)+2*sum(log(diag(L)));
        assert(value>=lo(j)-1e-10 && value<=hi(j)+1e-10);
    end
end
f=uavpriority.fit_position(uavpriority.subbundle(b,1:4),c);
assert(f.quality && ~f.sensitivity_ok); % no hidden interior derivative required
original=f.x;v=uavpriority.verify_position(f,b,c);assert(v.passed && v.estimate_unchanged);
assert(isequal(f.x,original) && isequal(v.fit_ids,1:4));
% Independently reconstruct the conditional distribution at the witness.
[d,~,C]=uavmvp.range_model(v.witness_xy_m',b.anchors,b.Pnav,b.Cradio,0);
K=C(5,1:4)/C(1:4,1:4);S=C(5,5)-K*C(1:4,5);
eta=b.ranges(5)-d(5)-K*(b.ranges(1:4)-d(1:4));
assert(abs(v.variance-S)<1e-10 && abs(v.residual-eta)<1e-10);
assert(S<C(5,5)); % shared bias/cross-covariance was conditioned, not discarded
bounded=b;bounded.model_bias_bounds_m=[.02;.03;.04;.01;.05];
bounded_fit=uavpriority.fit_position(uavpriority.subbundle(bounded,1:4),c);
bc=uavpriority.verify_position(bounded_fit,bounded,c);
[~,~,C]=uavmvp.range_model(bc.witness_xy_m',b.anchors,b.Pnav,b.Cradio,0);
K=C(5,1:4)/C(1:4,1:4);
signs=2*(dec2bin(0:31,5)-'0')-1;
errors=signs.*bounded.model_bias_bounds_m';
maximum=max(abs(errors(:,5)-errors(:,1:4)*K'));
assert(abs(maximum-bc.model_residual_bound_m)<1e-10);
bad=b;bad.ranges(5)=bad.ranges(5)+100;
rejected=uavpriority.verify_position(f,bad,c);assert(~rejected.passed);
assert(isequal(f.x,original) && isequal(f.record_ids,1:4));
% A constrained boundary optimum still has a usable global grid region.
edge_a=[90,20,15;110,20,15;90,60,15;110,60,15;100,40,15]';
edge=make_bundle(edge_a,[120;40]);ef=uavpriority.fit_position(uavpriority.subbundle(edge,1:4),c);
assert(ef.quality && ef.x(1)>119.99 && ~ef.sensitivity_ok);
assert(uavpriority.verify_position(ef,edge,c).passed);
% Four separated but near-collinear anchors cannot certify a huge region.
line_a=[20,10,15;30,10,15;40,10,15;50,10,15;35,20,15]';
line=make_bundle(line_a,[35;10]);line.Cradio=100*eye(5)+9*ones(5);
lf=uavpriority.fit_position(uavpriority.subbundle(line,1:4),c);
assert(~lf.quality && lf.radius>c.stage.region_radius_limit_m);
invalid=b;invalid.Cradio=-100*eye(5);
invalid_fit=uavpriority.fit_position(uavpriority.subbundle(invalid,1:4),c);
assert(~invalid_fit.quality && isinf(invalid_fit.radius));
small=uavstage.compact_fit(f);assert(~isfield(small,'region_mask') && ~isfield(small,'grid_nll'));
out=struct('all_passed',true,'mission_count',0,'box_bound_checked',true, ...
    'conditional_blocks_checked',true,'boundary_allowed_when_region_small',true, ...
    'bad_geometry_rejected',true,'bad_heldout_rejected',true, ...
    'fit_frozen_under_heldout_changes',true,'no_large_region_arrays_in_events',true);
end
function b=make_bundle(a,x)
b=struct('anchors',a,'ranges',vecnorm(a-[x;0])', ...
    'Pnav',.04*eye(15),'Cradio',eye(5)+9*ones(5), ...
    'record_ids',1:5,'nav_ids',1:5,'version',1, ...
    'model_bias_bounds_m',zeros(5,1),'aggregation_matrix',eye(5), ...
    'constituent_ids',{{1,2,3,4,5}});
b.Pnavigation=b.Pnav;b.Ptiming=zeros(15);
end
