function out=forest_motion(action,in,c)
%FOREST_MOTION M01: estimate-only command, separate plant derivative/contact.
% No world/task is advanced; nominal_step is an estimate-only local quadrature.
g=[0;0;-9.80665];m=c.motion_model;
switch action
    case 'nominal_step'
        n=in.navigation;dt=in.duration_s;w0=in.body_rate_radps(:);
        assert(dt>0 && dt<=c.initialization.imu_sample_interval_s*(1+1e-6));
        assert(numel(w0)==3 && all(isfinite(w0)) && norm(w0)<=m.body_rate_limit_rad_per_s*(1+1e-8));
        command=uavmvp.forest_motion('command',in,c);u=command.body_rate_rad_per_s;
        decay=exp(-dt/m.body_rate_time_constant_s);
        w1=u+(w0-u)*decay;
        meanRate=u+(w0-u)*(m.body_rate_time_constant_s/dt)*(1-decay);
        Rmid=n.R*uavmvp.SO3.exp(meanRate*dt/2);force=[0;0;command.normalized_thrust_mps2];
        acceleration=Rmid*force+g;
        next=n;next.p=n.p+n.v*dt+.5*acceleration*dt^2;next.v=n.v+acceleration*dt;
        next.R=n.R*uavmvp.SO3.exp(meanRate*dt);next.t=n.t+dt;
        out=struct('navigation',next,'body_rate_radps',w1,'command',command, ...
            'profile',struct('R',Rmid,'specific_force_body_mps2',force,'body_rate_radps',meanRate,'duration_s',dt));
    case 'command'
        n=in.navigation;q=in.setpoint_m(:);delta=q-n.p;
        vh=delta(1:2);d=norm(vh);v=vh/max(d,1e-12)*min(c.service.cruise_speed_m_per_s,sqrt(2*c.service.acceleration_m_per_s2*d));
        vz=sign(delta(3))*min(m.vertical_cruise_speed_limit_m_per_s,sqrt(2*m.vertical_acceleration_limit_m_per_s2*abs(delta(3))));
        if in.landing,vz=-c.landing.descent_speed_m_per_s;end
        acc=([v;vz]-n.v)/m.velocity_feedback_time_constant_s;
        acc(1:2)=acc(1:2)/max(1,norm(acc(1:2))/c.service.acceleration_m_per_s2);
        acc(3)=max(-m.vertical_acceleration_limit_m_per_s2,min(m.vertical_acceleration_limit_m_per_s2,acc(3)));
        z=(acc-g)/norm(acc-g);heading=[cos(m.nominal_yaw_command_rad);sin(m.nominal_yaw_command_rad);0];
        y=cross(z,heading);y=y/norm(y);Rdes=[cross(y,z),y,z];
        w=m.attitude_error_gain_per_s*uavmvp.SO3.log(n.R'*Rdes);w=w/max(1,norm(w)/m.body_rate_limit_rad_per_s);
        out=struct('body_rate_rad_per_s',w,'normalized_thrust_mps2',min(m.maximum_normalized_thrust_m_per_s2,norm(acc-g)), ...
            'estimated_arrived',norm(delta)<=m.estimated_arrival_position_tolerance_m && norm(n.v)<=m.estimated_arrival_speed_tolerance_m_per_s);
    case 'derivative'
        x=in.plant;u=in.command;
        out=struct('p_dot',x.v,'v_dot',x.R(:,3)*u.normalized_thrust_mps2+g, ...
            'R_dot',x.R*uavmvp.SO3.skew(x.omega), ...
            'omega_dot',(u.body_rate_rad_per_s-x.omega)/m.body_rate_time_constant_s);
    case 'contact'
        x=in.plant;hit=x.p(3)<=in.true_home_m(3) && x.v(3)<=0;
        if hit,x.p(3)=in.true_home_m(3);x.v=zeros(3,1);end
        out=struct('plant',x,'offline_physical_contact',hit);
    case 'landing'
        n=in.navigation;gate=uavmvp.forest_navigation('gate',n,c);
        assert(in.return_reservation_s>0 && in.next_reference_reservation_s>0);
        continueReference=n.t+in.next_reference_reservation_s<=in.reference_deadline_s;
        valid=gate.ok && continueReference && n.t+in.return_reservation_s<=c.service.task_resource_deadline_s;
        out=struct('admissible',valid,'setpoint_m',[0;0;n.p(3)-c.landing.descent_speed_m_per_s], ...
            'css_reference_required',true,'estimated_contact_candidate',n.p(3)<=0, ...
            'physical_landing_confirmed',false);
    case 'evaluate'
        % Offline-only; no result may be used by online state or policy.
        errors=vecnorm(in.estimated_terminal_xy_m-in.true_terminal_xy_m,2,2);
        returned=norm(in.final_true_position_m(:)-in.true_home_m(:))<=c.landing.actual_return_tolerance_m && in.physically_landed;
        out=struct('per_terminal_error_m',errors,'all_terminal_accuracy',all(errors<=c.evaluation.per_terminal_true_horizontal_error_limit_m), ...
            'actual_return',returned,'success',in.service_closed && in.all_physically_asleep && returned && all(errors<=c.evaluation.per_terminal_true_horizontal_error_limit_m));
    otherwise,error('forest:Action','Unknown motion action.');
end
end
