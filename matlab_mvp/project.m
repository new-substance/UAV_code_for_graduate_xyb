function out=project(action,options)
%PROJECT Single managed entry; task runs use the explicit review.json scope.
if nargin<1,action='status';end
setup_project();repo=fileparts(fileparts(mfilename('fullpath')));
review=jsondecode(fileread(fullfile(repo,'research_configs','review.json')));
switch lower(char(action))
    case 'status'
        out=struct('research_core','End-to-end task first; independent module innovations not required', ...
            'route_guidance','Terminal estimates guide following service waypoints; IMU/vision maintain UAV pose', ...
            'review_status',review.status,'mission_enabled',review.mission_enabled, ...
            'reason','Read review.json for allowed development scope; independent full-mission performance remains unvalidated', ...
            'specification','论文总纲.md','configuration','research_configs/baseline.json');
    case 'config'
        out=load_project_config();
    case 'check'
        out=deterministic_checks();
    case 'run'
        uavmvp.review_gate();
        assert(nargin==2,'uavproject:RunOptions','Declare seeds, count, rule and a new output label.');
        out=uavpriority.run_managed(options);
    otherwise
        error('uavproject:UnknownAction','Use status, config, check or run.');
end
if nargout==0,disp(out);end
end

function out=deterministic_checks()
% These are algebra/config/path checks, never MotionWorld or task replay.
c=load_project_config();x=[30;30];
a=[10,10,15;50,10,15;10,50,15;50,50,15]';
[d,H,C]=uavmvp.range_model(x,a,.04*eye(12),eye(4)+9*ones(4),0);
assert(all(isfinite(d)) && rank(H)==2 && all(eig(C)>0));
assert(all(abs(diag(C)-diag(C'))<1e-12));
assert(C(1,2)>8); % shared range bias is not treated as independent white noise
speed=299792458;alpha_a=1.00002;alpha_b=.99997;distance=37;delay=.008;hw=3e-9;
t=[0,.2];tau=[alpha_a*t;alpha_b*(t+distance/speed); ...
    alpha_b*(t+distance/speed+delay);alpha_a*(t+2*distance/speed+delay+hw)];
r=uavmvp.rtt_solve(tau,alpha_a,hw,[],zeros(8));
assert(abs(r.range_m-distance)<1e-5 && abs(r.kappa-alpha_a/alpha_b)<1e-10);
packages={'uavmvp.range_model','uavstage.fit_position','uavpriority.fit_position'};
main=fileparts(mfilename('fullpath'));
for k=1:numel(packages)
    found=which(packages{k},'-all');if ischar(found),found={found};end
    assert(numel(found)==1 && startsWith(found{1},main,'IgnoreCase',true), ...
        'uavproject:PathAmbiguity','Start a clean MATLAB session and load only matlab_mvp.');
end
try
    uavmvp.review_gate();
catch problem
    assert(ismember(problem.identifier,{'uavproject:SimulationNotApproved','uavproject:ReviewMismatch'}));
end
original_path=path;restore_path=onCleanup(@()path(original_path)); %#ok<NASGU>
addpath(fullfile(fileparts(main),'research_checks'));
writer=check_bounded_writer();feedback=check_position_feedback();state=check_localization_state();
region=check_region_quality();
returning=check_return_reservation();
coordination=check_coordination();
marginal=check_forecast_marginal();
joint=check_joint_baseline();
horizon=check_noise_horizon();
out=struct('all_passed',writer.all_passed && feedback.all_passed && state.all_passed && region.all_passed && returning.all_passed && coordination.all_passed && marginal.all_passed && joint.all_passed && horizon.all_passed, ...
    'kind','deterministic_algebra_paths_run_gate_writer_and_feedback', ...
    'mission_count',0,'rtt_error_m',abs(r.range_m-distance),'source_roots',1, ...
    'writer',writer,'position_feedback',feedback,'localization_state',state,'region_quality',region, ...
    'return_reservation',returning,'coordination',coordination,'forecast_marginal',marginal,'joint_baseline',joint, ...
    'noise_horizon',horizon);
end
