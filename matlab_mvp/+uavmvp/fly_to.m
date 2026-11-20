function status=fly_to(world,target,cfg,max_duration)
%FLY_TO Integrate physical motion driven only by the public nav estimate.
assert(numel(target)==3 && max_duration>=0);start=world.t;
target=target(:);status=struct('arrived',false,'reason','MOVEMENT_TIMEOUT','duration_s',0);
while world.t-start<max_duration-1e-10
    nav=world.nav;
    if norm(nav.p-target)<=.25 && norm(nav.v)<=.10
        status.arrived=true;status.reason='ESTIMATED_ARRIVAL';break;
    end
    a=uavmvp.control_acceleration(nav,target,cfg);
    world.advance(min(.05,max_duration-(world.t-start)),a);
end
status.duration_s=world.t-start;
% Arrival by estimate is NOT proof of true position or return success.
end
