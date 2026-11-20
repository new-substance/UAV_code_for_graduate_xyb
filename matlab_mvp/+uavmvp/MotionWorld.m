classdef MotionWorld < handle
    %MOTIONWORLD Private kinematic truth; IMU and visible marker observations.
    % Ideal low-level acceleration actuation, NOT a motor/flight-control model.
    % Task commands use estimated position/velocity; truth is exportable only
    % after finish(), which irreversibly disables further mission operations.
    properties (SetAccess=private)
        nav
        t=0
        finished=false
    end
    properties (Access=private)
        cfg
        truth
        home_true
        path
        seed
        trace=zeros(0,16)
        reference_log={}
        airborne=false
        grounded=true
        contact_log=zeros(0,5)
        radio=[]
        radio_log={}
        radio_imu_time_offset_s
    end
    methods
        function obj=MotionWorld(cfg,seed)
            obj.cfg=cfg;obj.seed=seed;obj.nav=uavmvp.nav_init(cfg);
            rs=RandStream('mt19937ar','Seed',seed);
            error=uavmvp.initial_error(obj.nav.P,rs);
            obj.truth=uavmvp.nav_inject(obj.nav,error);
            if isfield(cfg.sim,'truth_frame_translation_m')
                shift=cfg.sim.truth_frame_translation_m(:);assert(numel(shift)==3);
                obj.truth.p=obj.truth.p+shift;
                obj.truth.map_p=obj.truth.map_p+shift;
            end
            % The launch pad is stationary. Keep the uncertain velocity prior
            % as estimation error instead of physically sliding at t=0.
            obj.truth.v=zeros(3,1);obj.nav.v=-error(4:6);
            obj.home_true=obj.truth.p;
            timing_rs=RandStream('mt19937ar','Seed',mod(seed+65537,4294967291));
            obj.radio_imu_time_offset_s=cfg.navigation_candidate.timestamp_alignment_std_s*randn(timing_rs);
            obj.path=uavmvp.NoisePath(max(1200,cfg.motion_and_budget.time_limit_s+600), ...
                cfg.sim.dt,12,mod(seed+314159,2^32-1));
            obj.record();
        end
        function advance(obj,duration,acceleration,yaw_rate)
            assert(~obj.finished && duration>=0 && isfinite(duration));
            if nargin<3,acceleration=zeros(3,1);end
            if nargin<4,yaw_rate=0;end
            target=obj.t+duration;cfg=obj.cfg;n=cfg.navigation_candidate;
            while target-obj.t>1e-12
                next_tick=(floor((obj.t+1e-10)/cfg.sim.dt)+1)*cfg.sim.dt;
                dt=min(target-obj.t,next_tick-obj.t);
                assert(dt>0);a=acceleration(:);
                amax=cfg.motion_and_budget.horizontal_acceleration_mps2;
                a(1:2)=a(1:2)/max(1,norm(a(1:2))/amax);a(3)=max(-amax,min(amax,a(3)));
                vn=obj.truth.v+a*dt;
                vn(1:2)=vn(1:2)/max(1,norm(vn(1:2))/cfg.motion_and_budget.horizontal_max_speed_mps);
                vn(3)=min(cfg.motion_and_budget.vertical_speed_mps,max(-cfg.motion_and_budget.vertical_speed_mps,vn(3)));
                pn=obj.truth.p+(obj.truth.v+vn)*dt/2;
                if pn(3)<=obj.home_true(3) && vn(3)<=0
                    if obj.airborne && ~obj.grounded
                        obj.contact_log(end+1,:)=[obj.t+dt,pn(1:2)',abs(vn(3)),norm(vn(1:2))];
                    end
                    % Flat contact surface through the actual launch pad.
                    % Ground contact/friction is physics, never a nav update.
                    pn(3)=obj.home_true(3);vn=zeros(3,1);
                    if obj.truth.p(3)<=obj.home_true(3)+1e-10,pn(1:2)=obj.truth.p(1:2);end
                    obj.grounded=true;
                else
                    obj.grounded=false;
                end
                if pn(3)>obj.home_true(3)+.25,obj.airborne=true;end
                a=(vn-obj.truth.v)/dt; % includes the discrete contact impulse
                omega=[0;0;yaw_rate];Rm=obj.truth.R*uavmvp.SO3.exp(omega*dt/2);
                dw=obj.path.delta(obj.t,obj.t+dt);
                dba=n.acc_bias_random_walk_mps2_per_sqrt_s*dw(7:9);
                dbg=n.gyro_bias_random_walk_radps_per_sqrt_s*dw(10:12);
                imu.acc=Rm'*(a-cfg.sim.gravity)+obj.truth.ba+dba/2+ ...
                    n.acc_noise_density_mps2_per_sqrt_hz*dw(1:3)/dt;
                imu.gyro=omega+obj.truth.bg+dbg/2+ ...
                    n.gyro_noise_density_radps_per_sqrt_hz*dw(4:6)/dt;
                obj.truth.p=pn;
                obj.truth.v=vn;obj.truth.R=obj.truth.R*uavmvp.SO3.exp(omega*dt);
                obj.truth.ba=obj.truth.ba+dba;obj.truth.bg=obj.truth.bg+dbg;
                obj.nav=uavmvp.nav_predict(obj.nav,imu,dt,cfg);
                obj.t=obj.t+dt;
            end
            obj.t=target;obj.nav.t=target;obj.record();
        end
        function hold(obj,duration)
            target=obj.nav.p;end_time=obj.t+duration;
            while end_time-obj.t>1e-12
                a=uavmvp.control_acceleration(obj.nav,target,obj.cfg);
                obj.advance(min(.05,end_time-obj.t),a);
            end
        end
        function [observation,report]=observe_reference(obj,id)
            assert(~obj.finished);cfg=obj.cfg;
            % Reference acquisition time belongs to the actual timeline.
            obj.hold(cfg.navigation_candidate.reference_observation_s);
            [pixels,~,visible]=uavmvp.camera_model(obj.truth,id,cfg);
            rs=obj.stream(51,id);detected=visible && rand(rs)<cfg.navigation_candidate.marker_detect_probability_given_visibility;
            forced_outage=false;
            if isfield(cfg.sim,'reference_outage_intervals_s')
                intervals=cfg.sim.reference_outage_intervals_s;
                forced_outage=any(obj.t>=intervals(:,1) & obj.t<intervals(:,2));
                detected=detected && ~forced_outage;
            end
            if detected
                observation=struct('marker_id',id,'detected',true, ...
                    'epoch_s',obj.t,'pixels',pixels+cfg.navigation_candidate.camera.pixel_std* ...
                    cfg.sim.camera_noise_scale*randn(rs,8,1));
            else
                observation=[];
            end
            prior=obj.nav;
            [obj.nav,report]=uavmvp.nav_update_reference(obj.nav,observation,cfg);
            % Replayable sensor/update evidence; exported only after finish.
            obj.reference_log{end+1}=struct('time_s',obj.t,'marker_id',id, ...
                'observation',observation,'prior',prior,'report',report, ...
                'posterior_p',obj.nav.p,'posterior_R',obj.nav.R, ...
                'true_p',obj.truth.p,'true_R',obj.truth.R);
            if isfield(cfg.sim,'reference_outage_intervals_s')
                obj.reference_log{end}.sensor_forced_outage=forced_outage;
            end
            obj.record();
        end
        function [id,raw]=clone_antenna(obj)
            assert(~obj.finished);[obj.nav,id,raw]=uavmvp.nav_clone(obj.nav,obj.cfg);
            % First-order association error, shared over the complete flight.
            % Not a perturbation to the physical trajectory or current INS.
            offset=obj.nav.v*obj.radio_imu_time_offset_s;
            obj.nav.clones(:,end)=obj.nav.clones(:,end)+offset;raw.position=raw.position+offset;
        end
        function prune(obj,ids)
            assert(~obj.finished);obj.nav=uavmvp.nav_prune(obj.nav,ids);
        end
        function configure_radio(obj,count,mission,fixture)
            assert(~obj.finished && isempty(obj.radio));
            if nargin<4,fixture=[];end
            obj.radio=uavmvp.RadioWorld(obj.cfg,mod(obj.seed+271828,4294967291),count,mission,fixture);
        end
        function packets=transmit_group(obj,control)
            assert(~obj.finished && ~isempty(obj.radio));packets={};
            if isempty(control),return;end
            start=obj.t;timing=uavmvp.radio_timing(obj.cfg,numel(control.active_members));
            pending=cell(1,numel(control.active_members));delays=zeros(size(pending));
            obj.hold(control.frame_s);
            for k=1:numel(pending)
                id=control.active_members(k);delays(k)=obj.radio.propagation(id,obj.antenna_truth());
                if obj.radio.decode(id,'control',obj.t,obj.antenna_truth()),pending{k}=obj.radio.group(id,control);end
            end
            for k=1:numel(pending)
                id=control.active_members(k);slot=control.schedule(k);
                receive=start+control.frame_s+delays(k)+slot.reply_offset_s+timing.frames.identity;
                obj.hold(max(0,receive-obj.t));
                if ~isempty(pending{k})
                    obj.hold(obj.radio.propagation(id,obj.antenna_truth()));
                    if obj.radio.decode(id,'identity',obj.t,obj.antenna_truth())
                        packet=pending{k};packet.received_time_s=obj.t;packets{end+1}=packet; %#ok<AGROW>
                    end
                end
            end
            obj.hold(max(0,start+control.reserved_s-obj.t));
            obj.radio_log{end+1}=struct('kind',control.kind,'start_s',start,'end_s',obj.t, ...
                'members',control.active_members,'received_count',numel(packets));
        end
        function raw=measure_service(obj,request)
            assert(~obj.finished && ~isempty(obj.radio));start=obj.t;id=request.address;
            assert(abs(start-request.measurement_epoch_s)<1e-8);
            [~,~,~]=uavmvp.nav_archive(obj.nav,request.nav_snapshot_id);
            timing=uavmvp.radio_timing(obj.cfg);count=request.exchange_count;
            stamps=zeros(4,count);support=[];ranges=[];reason='VALID';
            raw=[];
            for exchange=1:count
                if exchange>1,obj.hold(max(0,start+obj.cfg.css.cold_request_min_separation_s-obj.t));end
                t1=obj.t;
                if exchange==1,clone=request.nav_snapshot_id;else,[clone,~]=obj.clone_antenna();end
                support(end+1)=clone;ranges(end+1)=obj.radio.range_truth(id,obj.antenna_truth()); %#ok<AGROW>
                t2=t1+obj.radio.propagation(id,obj.antenna_truth());
                allowed=obj.radio.range_allowed(id,request) && obj.radio.decode(id,'request',t1,obj.antenna_truth());
                t3=t2+timing.reply_interval_s;obj.hold(t3-obj.t);
                if ~allowed
                    obj.hold(max(0,t1+obj.cfg.css.exchange_deadline_s-obj.t));reason='NO_RANGE_RESPONSE';break;
                end
                t4=t3+obj.radio.propagation(id,obj.antenna_truth());obj.hold(t4-obj.t);
                [clone,~]=obj.clone_antenna();support(end+1)=clone; %#ok<AGROW>
                ranges(end+1)=obj.radio.range_truth(id,obj.antenna_truth()); %#ok<AGROW>
                stamps(:,exchange)=obj.radio.timestamp(id,[t1;t2;t3;t4]);
                obj.hold(timing.frames.range_response);
                response=obj.radio.decode(id,'response',obj.t,obj.antenna_truth());
                meta_start=t3+timing.frames.range_response+obj.cfg.css.metadata_processing_switch_s+obj.cfg.css.guard_s;
                obj.hold(max(0,meta_start-obj.t));
                meta_end=obj.t+obj.radio.propagation(id,obj.antenna_truth())+timing.frames.range_metadata;
                obj.hold(meta_end-obj.t);
                metadata=obj.radio.decode(id,'metadata',obj.t,obj.antenna_truth());
                obj.hold(obj.cfg.css.guard_s);
                if ~response || ~metadata
                    obj.hold(max(0,t1+obj.cfg.css.exchange_deadline_s-obj.t));reason='INCOMPLETE_RANGE_METADATA';break;
                end
            end
            if strcmp(reason,'VALID')
                [positions,P]=uavmvp.nav_archive(obj.nav,support);
                weights=ones(numel(support),1)/numel(support);
                [obj.nav,composite,mean_raw]=uavmvp.nav_combine_clones(obj.nav,support,weights);
                % Relative-position uncertainty, not absolute navigation error,
                % screens the short-stop approximation (independent of truth).
                meanJ=kron(weights',eye(3));spread=0;pair_bounds=zeros(1,count);
                for k=1:numel(support)
                    J=-meanJ;J(:,3*k-2:3*k)=J(:,3*k-2:3*k)+eye(3);
                    relative=J*P*J';spread=max(spread,norm(positions(:,k)-mean_raw.position)+3*sqrt(max(eig(relative))));
                end
                for k=1:count
                    J=zeros(3,size(P,1));J(:,6*k-5:6*k-3)=-eye(3);J(:,6*k-2:6*k)=eye(3);
                    pair_bounds(k)=norm(positions(:,2*k)-positions(:,2*k-1))+3*sqrt(max(0,max(eig(J*P*J'))));
                end
                height_floor=mean_raw.position(3)-obj.cfg.scene.terminal_height_m-3*sqrt(mean_raw.covariance(3,3))-spread;
                bound=Inf;if height_floor>0,bound=spread^2/(2*height_floor);end
                raw=struct('timestamps',stamps,'received_time_s',obj.t,'effective_nav_snapshot_id',composite, ...
                    'effective_epoch_s',mean_raw.epoch_s,'support_snapshot',mean_raw, ...
                    'support_spread_bound_m',spread,'range_curvature_bound_m',bound, ...
                    'exchange_displacement_bounds_m',pair_bounds, ...
                    'motion_accepted',spread<=obj.cfg.sim.range_support_span_limit_m && bound<=obj.cfg.sim.range_model_bias_bound_limit_m);
            end
            obj.prune(setdiff(obj.nav.clone_ids,support));
            obj.radio_log{end+1}=struct('kind','RANGE_SERVICE','address',id,'mode',request.mode, ...
                'start_s',start,'end_s',obj.t,'reason',reason,'support_ids',support, ...
                'true_support_ranges_m',ranges,'request',request,'raw',raw);
        end
        function packet=transmit_stop(obj,command)
            assert(~obj.finished && ~isempty(obj.radio));packet=[];start=obj.t;id=command.address;
            timing=uavmvp.radio_timing(obj.cfg);obj.hold(timing.frames.stop);
            obj.hold(obj.radio.propagation(id,obj.antenna_truth()));
            if obj.radio.decode(id,'stop',obj.t,obj.antenna_truth()),reply=obj.radio.stop(id,command);else,reply=[];end
            obj.hold(obj.cfg.css.guard_s+timing.frames.stop_ack);
            obj.hold(obj.radio.propagation(id,obj.antenna_truth()));
            if ~isempty(reply) && obj.radio.decode(id,'ack',obj.t,obj.antenna_truth())
                packet=reply;obj.hold(obj.cfg.css.guard_s);packet.received_time_s=obj.t;
            else
                obj.hold(max(0,start+command.reserved_s-obj.t));
            end
            obj.radio_log{end+1}=struct('kind','STOP','address',id,'start_s',start,'end_s',obj.t, ...
                'physically_received',~isempty(reply),'ack_received',~isempty(packet));
        end
        function data=finish(obj)
            assert(~obj.finished);obj.finished=true;obj.record();
            data=struct('time_s',obj.t,'true_position',obj.truth.p,'estimated_position',obj.nav.p, ...
                'true_home',obj.home_true,'return_error_m',norm(obj.truth.p-obj.home_true), ...
                'navigation_error_m',norm(obj.truth.p-obj.nav.p),'trajectory',obj.trace, ...
                'reference_log',{obj.reference_log}, ...
                'physically_landed',obj.airborne && abs(obj.truth.p(3)-obj.home_true(3))<1e-8 && norm(obj.truth.v)<1e-6, ...
                'ground_contact_events',obj.contact_log, ...
                'ground_contact_columns',{{'t','true_x','true_y','vertical_speed_mps','horizontal_speed_mps'}}, ...
                'trajectory_columns',{{'t','true_x','true_y','true_z','nav_x','nav_y','nav_z', ...
                    'true_vx','true_vy','true_vz','nav_vx','nav_vy','nav_vz','sigma_x','sigma_y','sigma_z'}});
            data.radio_events=obj.radio_log;
            data.radio_imu_time_offset_s=obj.radio_imu_time_offset_s;
            if ~isempty(obj.radio),data.radio_truth=obj.radio.offline();end
        end
    end
    methods (Access=private)
        function p=antenna_truth(obj)
            p=obj.truth.p+obj.truth.R*obj.cfg.navigation_candidate.antenna_lever_arm_body_m(:);
        end
        function rs=stream(obj,kind,id)
            tick=round(obj.t*1e6);
            seed=mod(double(obj.seed)*131071+kind*524287+id*8191+tick*17,4294967291);
            rs=RandStream('mt19937ar','Seed',seed);
        end
        function record(obj)
            obj.trace(end+1,:)=[obj.t,obj.truth.p',obj.nav.p',obj.truth.v',obj.nav.v', ...
                sqrt(max(0,diag(obj.nav.P(1:3,1:3))))'];
        end
    end
end
