function I=ranging_information(H,cfg)
%RANGING_INFORMATION P01 known common-bias template for proposal ranking only.
assert(size(H,2)==2 && all(isfinite(H),'all'));
n=size(H,1);s2=cfg.ranging.range_white_std_m^2/cfg.priority.cold_services_per_position;
b2=cfg.ranging.device_persistent_bias_std_m^2;
assert(isfinite(s2) && s2>0 && isfinite(b2) && b2>=0);
v=sum(H,1);I=(H'*H)/s2-b2*(v'*v)/(s2*(s2+n*b2));I=(I+I')/2;
end
