function [xy,detail]=next_extra(nav,fit,visited,cfg)
%NEXT_EXTRA Fixed visual-footprint candidates; no future ranging or truth.
xy=[];detail=struct('available',false,'reference_id',NaN,'score',Inf);
if isempty(fit) || any(~isfinite(fit.x)) || ~isfinite(nav.last_reference_id),return;end
points=uavpriority.inset_candidates(cfg);
for k=1:size(points,1)
    if ~isempty(visited) && min(vecnorm(visited-points(k,:),2,2))< ...
            cfg.localization.distinct_horizontal_separation_m+1,continue;end
    distance=norm([points(k,:)';cfg.scene.flight_height_m]-[fit.x;cfg.scene.terminal_height_m]);
    if distance>cfg.scene.design_service_slant_m,continue;end
    path=uavmvp.reference_path(cfg,nav.last_reference_id,k,[]);
    if ~path.available,continue;end
    score=path.time_proxy_s+2*cfg.priority.inset_service_offset_m/cfg.motion_and_budget.horizontal_max_speed_mps+.05*distance;
    if score<detail.score
        xy=points(k,:);detail=struct('available',true,'reference_id',k,'score',score);
    end
end
end
