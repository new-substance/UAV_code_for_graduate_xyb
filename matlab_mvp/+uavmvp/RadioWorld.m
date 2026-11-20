classdef RadioWorld < handle
    %RADIOWORLD Private physical radio/terminal state, owned by MotionWorld.
    % Only the owner passes physical positions; online code sees decoded data.
    properties (Access=private)
        cfg
        seed
        positions
        biases
        devices
        clocks
        bad_intervals
    end
    methods
        function obj=RadioWorld(cfg,seed,count,mission,fixture)
            if nargin<5,fixture=[];end
            obj.cfg=cfg;obj.seed=seed;obj.positions=zeros(3,count);obj.biases=zeros(1,count);
            obj.devices=cell(1,count);obj.clocks=cell(1,count);obj.bad_intervals=cell(1,count);
            horizon=max(1200,cfg.motion_and_budget.time_limit_s+600);
            for id=1:count
                rs=RandStream('mt19937ar','Seed',mod(seed+104729*id,4294967291));
                obj.positions(:,id)=[cfg.scene.width_m*rand(rs);cfg.scene.height_m*rand(rs);cfg.scene.terminal_height_m];
                bias_std=cfg.ranging.device_persistent_bias_std_m;
                if isfield(cfg.sim,'radio_truth_device_bias_std_m'),bias_std=cfg.sim.radio_truth_device_bias_std_m;end
                obj.biases(id)=bias_std*randn(rs);
                if isfield(cfg.sim,'terminal_device_class')
                    obj.devices{id}=feval(cfg.sim.terminal_device_class,id,mission);
                else
                    obj.devices{id}=uavmvp.TerminalDevice(id,mission);
                end
                obj.clocks{id}=uavmvp.ClockPath(cfg,horizon,mod(seed+130363*id,4294967291));
                probability=cfg.ranging.stress.link_bad_state_probability;
                bad=rand(rs)<probability;t=0;intervals=zeros(0,2);
                while t<horizon
                    mean_duration=cfg.ranging.stress.bad_state_mean_duration_s;
                    if ~bad,mean_duration=mean_duration*(1-probability)/max(probability,eps);end
                    next=t-mean_duration*log(max(realmin,rand(rs)));
                    if bad,intervals(end+1,:)=[t,next];end %#ok<AGROW>
                    t=next;bad=~bad;
                end
                obj.bad_intervals{id}=intervals;
            end
            if ~isempty(fixture)
                assert(all(size(fixture)==[3,count]));obj.positions=fixture;
            end
            if isfield(cfg.sim,'truth_frame_translation_m')
                obj.positions=obj.positions+cfg.sim.truth_frame_translation_m(:);
            end
        end
        function delay=propagation(obj,id,antenna)
            distance=norm(antenna-obj.positions(:,id));
            delay=distance/299792458+obj.cfg.sim.radio_hardware_delay_s/2+obj.biases(id)/299792458;
            assert(delay>0,'uavmvp:physicalDelay','Hardware residual exceeds declared physical delay.');
        end
        function ok=decode(obj,id,kind,time,antenna)
            ok=false;
            if norm(antenna-obj.positions(:,id))>obj.cfg.scene.design_service_slant_m,return;end
            p=obj.cfg.ranging.baseline_stage_probabilities;
            switch kind
                case 'control',probability=p.wake_decode;code=11;
                case 'identity',probability=p.identity_decode_given_control;code=13;
                case 'request',probability=p.range_exchange_valid^(1/3);code=17;
                case 'response',probability=p.range_exchange_valid^(1/3);code=19;
                case 'metadata',probability=p.range_exchange_valid^(1/3);code=23;
                case 'stop',probability=p.wake_decode;code=29;
                case 'ack',probability=p.identity_decode_given_control;code=31;
                otherwise,error('uavmvp:radioFrame','Unknown frame type.');
            end
            if obj.cfg.sim.radio_correlated_bad_states
                intervals=obj.bad_intervals{id};
                if any(time>=intervals(:,1) & time<intervals(:,2)),probability=.05*probability;end
            end
            rs=obj.stream(id,code,time);ok=rand(rs)<probability;
        end
        function packet=group(obj,id,control)
            packet=obj.devices{id}.receive_group(control);
        end
        function ok=range_allowed(obj,id,request)
            ok=obj.devices{id}.can_respond_range(request);
        end
        function packet=stop(obj,id,command)
            packet=obj.devices{id}.receive_stop(command);
        end
        function stamps=timestamp(obj,id,times)
            % A clock is the known local time scale. B has unknown phase/rate.
            stamps=times(:);stamps(2:3)=obj.clocks{id}.sample(times(2:3));
            rs=obj.stream(id,37,times(1));sigma_m=obj.cfg.ranging.range_white_std_m;
            if isfield(obj.cfg.sim,'radio_truth_range_white_std_m'),sigma_m=obj.cfg.sim.radio_truth_range_white_std_m;end
            sigma=sigma_m/299792458;
            stamps=stamps+sigma*randn(rs,4,1);
        end
        function distance=range_truth(obj,id,antenna)
            distance=norm(antenna-obj.positions(:,id));
        end
        function data=offline(obj)
            data=struct('terminal_positions',obj.positions,'persistent_range_bias_m',obj.biases, ...
                'bad_intervals',{obj.bad_intervals});
            if isfield(obj.cfg.sim,'terminal_device_class')
                data.device_states=cellfun(@(d)d.offline_state(),obj.devices,'UniformOutput',false);
            end
        end
    end
    methods (Access=private)
        function rs=stream(obj,id,kind,time)
            seed=mod(double(obj.seed)*131071+id*8191+kind*524287+floor(time*1e6)*17,4294967291);
            rs=RandStream('mt19937ar','Seed',seed);
        end
    end
end
