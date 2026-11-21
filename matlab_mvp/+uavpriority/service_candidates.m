function [xy,references]=service_candidates(cfg)
%SERVICE_CANDIDATES P01 common finite centers/insets/outsets, no node truth.
base=cfg.scene.reference_centers_xy_m;inside=uavpriority.inset_candidates(cfg);
direction=base-inside;direction=direction./vecnorm(direction,2,2);
extent=[cfg.scene.width_m,cfg.scene.height_m];outside=zeros(size(base));
for k=1:size(base,1)
    limit=Inf;
    for axis=1:2
        if direction(k,axis)>0,reach=(extent(axis)-base(k,axis))/direction(k,axis);
        elseif direction(k,axis)<0,reach=-base(k,axis)/direction(k,axis);
        else,continue;
        end
        limit=min(limit,reach);
    end
    distance=min(cfg.priority.inset_service_offset_m,limit/2);
    assert(isfinite(distance) && distance>0);
    outside(k,:)=base(k,:)+distance*direction(k,:);
end
xy=[base;inside;outside];references=repmat((1:size(base,1))',3,1);
end
