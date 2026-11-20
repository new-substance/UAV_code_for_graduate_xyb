function out=project(action)
%PROJECT Single managed entry. No mission executor is released in this phase.
if nargin<1,action='status';end
setup_project();repo=fileparts(fileparts(mfilename('fullpath')));
review=jsondecode(fileread(fullfile(repo,'research_configs','review.json')));
switch lower(char(action))
    case 'status'
        out=struct('research_core','C3; C2 foundation; C1 engineering support', ...
            'review_status',review.status,'mission_enabled',false, ...
            'reason','Scope review and bounded lightweight writer remain pending', ...
            'specification','论文总纲.md','configuration','research_configs/baseline.json');
    case 'config'
        out=load_project_config();
    case 'check'
        out=deterministic_checks();
    case 'run'
        error('uavproject:SimulationNotApproved', ...
            'New missions are not approved. Complete the scoped model review and bounded writer first.');
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
    project('run');error('uavproject:GateFailed','A mission unexpectedly passed the gate.');
catch problem
    assert(strcmp(problem.identifier,'uavproject:SimulationNotApproved'));
end
out=struct('all_passed',true,'kind','deterministic_algebra_config_paths_and_run_gate', ...
    'mission_count',0,'rtt_error_m',abs(r.range_m-distance),'source_roots',1);
end
