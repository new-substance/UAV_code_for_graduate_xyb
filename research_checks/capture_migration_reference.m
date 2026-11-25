function report=capture_migration_reference(output_file)
% Deterministic fixed-input regression only: no MotionWorld or mission run.
repo=fileparts(fileparts(mfilename('fullpath')));
main=fullfile(repo,'matlab_mvp');
if isfile(fullfile(main,'setup_project.m'))
    addpath(main);setup_project();
else
    addpath(main,fullfile(repo,'single_terminal_stage'),fullfile(repo,'priority_stage'));
end
c=load_project_config('legacy');
a=[10,10,15;50,10,15;10,50,15;50,50,15;30,10,15]';
x=[30;30];r=vecnorm(a-[x;0])'+[.1;-.1;.05;-.05;.02];
b=struct('anchors',a,'ranges',r,'Pnav',.04*eye(15), ...
    'Cradio',eye(5)+9*ones(5),'record_ids',1:5,'nav_ids',1:5, ...
    'version',1,'model_bias_bounds_m',zeros(5,1));
b.Pnavigation=b.Pnav;b.Ptiming=zeros(15);b.aggregation_matrix=eye(5);
b.constituent_ids={1,2,3,4,5};
[d,H,C,G]=uavmvp.range_model(x,a,b.Pnav,b.Cradio,0);
fit=uavpriority.fit_position(uavpriority.subbundle(b,1:4),c);
verify=uavpriority.verify_position(fit,b,c);
alpha_a=1.00002;alpha_b=.99997;distance=37;delay=.008;hw=3e-9;speed=299792458;
t=[0,.2];stamps=[alpha_a*t;alpha_b*(t+distance/speed); ...
    alpha_b*(t+distance/speed+delay);alpha_a*(t+2*distance/speed+delay+hw)];
radio=uavmvp.rtt_solve(stamps,alpha_a,hw,[],1e-18*eye(8));
nav=uavmvp.nav_init(c);nav.p=[10;10;15];
[pixels,pixel_H,visible,detail]=uavmvp.camera_model(nav,1,c);
report=struct('kind','deterministic_fixed_input_no_mission', ...
    'config',c,'ranges',d,'H',H,'C',C,'G',G,'fit_x',fit.x, ...
    'fit_radius',fit.radius,'fit_objective',fit.objective, ...
    'fit_sensitivity',fit.sensitivity,'fit_quality',fit.quality,'fit_reason',fit.reason, ...
    'verification',verify,'radio',radio,'pixels',pixels, ...
    'pixel_H_nonzero',pixel_H(:,any(pixel_H,1)),'visible',visible,'pixel_detail',detail);
parent=fileparts(output_file);if ~isempty(parent) && ~isfolder(parent),mkdir(parent);end
uavmvp.write_json(output_file,report);
fprintf('Fixed-input migration reference saved; no simulation executed.\n');
end
