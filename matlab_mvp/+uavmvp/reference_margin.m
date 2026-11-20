function report=reference_margin(nav,id,cfg)
%REFERENCE_MARGIN Nominal projected visibility plus a local pixel envelope.
% Online-only screening heuristic; not a guaranteed capture probability.
[pixels,H,visible,details]=uavmvp.camera_model(nav,id,cfg);
C=H*nav.P*H';sigma=sqrt(max(0,diag(C)));
bounds=repmat([cfg.navigation_candidate.camera.width_px; ...
    cfg.navigation_candidate.camera.height_px],4,1);
margin=min([pixels-3*sigma;bounds-pixels-3*sigma]);
report=struct('nominal_visible',visible,'pixel_margin',margin, ...
    'envelope_inside',visible && isfinite(margin) && margin>=0,'geometry',details);
end
