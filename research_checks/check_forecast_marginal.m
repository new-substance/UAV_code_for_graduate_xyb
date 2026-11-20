function out=check_forecast_marginal()
%CHECK_FORECAST_MARGINAL Fixed coupled clones: keep exact UAV/map prediction.
repo=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(repo,'matlab_mvp'));
setup_project();c=load_project_config();n=uavmvp.nav_init(c);n.p=[10;10;15];n.last_reference_id=1;
for k=1:32,[n,~]=uavmvp.nav_clone(n,c);end
original=n;m=uavmvp.nav_prune(n,[]);d=size(m.P,1);ix=1:d;
assert(norm(n.P(1:d,d+1:end),'fro')>0); % clones genuinely correlated
full=uavmvp.predict_leg(n,[11;10;15],c,15,1);
small=uavmvp.predict_leg(m,[11;10;15],c,15,1);
assert(full.arrived==small.arrived && full.envelope_inside_area==small.envelope_inside_area);
assert(abs(full.duration_s-small.duration_s)<1e-12);
err=max(abs(full.nav.P(ix,ix)-small.nav.P),[],'all');assert(err<1e-11);
assert(norm(full.nav.p-small.nav.p)<1e-12);
assert(abs(full.reference.pixel_margin-small.reference.pixel_margin)<1e-9);
fullhold=uavmvp.predict_hold(full.nav,.75,c);
smallhold=uavmvp.predict_hold(small.nav,.75,c);
assert(max(abs(fullhold.nav.P(ix,ix)-smallhold.nav.P),[],'all')<1e-11);
back=uavmvp.reference_return_plan(fullhold.nav,c,1);
raw=uavmvp.predict_leg(fullhold.nav,[10;10;15],c,60,1);
assert(max(abs(back.first_leg.nav.P-raw.nav.P(ix,ix)),[],'all')<1e-11);
assert(back.first_leg.arrived==raw.arrived && abs(back.first_leg.duration_s-raw.duration_s)<1e-12);
g1=uavpriority.navigation_gate(n,1,c);g2=uavpriority.navigation_gate(m,1,c);
assert(isequaln(g1,g2));assert(isequaln(n,original));
out=struct('all_passed',true,'mission_count',0,'full_dimension',size(n.P,1), ...
    'forecast_dimension',d,'max_covariance_difference',err,'original_joint_state_unchanged',true);
end
