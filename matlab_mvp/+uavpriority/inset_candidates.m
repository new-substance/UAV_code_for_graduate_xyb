function xy=inset_candidates(cfg)
refs=cfg.scene.reference_centers_xy_m;
xmin=min(refs(:,1));xmax=max(refs(:,1));ymin=min(refs(:,2));ymax=max(refs(:,2));
direction=[double(refs(:,1)==xmin)-double(refs(:,1)==xmax), ...
    double(refs(:,2)==ymin)-double(refs(:,2)==ymax)];
direction=direction./vecnorm(direction,2,2);
xy=refs+cfg.priority.inset_service_offset_m*direction;
end
