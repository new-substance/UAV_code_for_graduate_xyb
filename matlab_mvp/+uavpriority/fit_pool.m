function [fit,chosen,selection]=fit_pool(records,nav,history,macros,pool,required,cfg,previous)
%FIT_POOL Four-record geometry selection, only after a round has ended.
full_bundle=uavpriority.bundle(records,nav,history,macros,pool,cfg);
probe=[cfg.scene.width_m;cfg.scene.height_m]/2;
if ~isempty(previous) && all(isfinite(previous.x),'all'),probe=previous.x;end
full_bundle=uavpriority.selection_bundle(full_bundle,required,cfg);
pick=cfg;pick.localization.group_geometry_rule='global_geometry';
pick.localization.region_chi2_df2_level=cfg.stage.region_gamma;
[chosen,selection]=uavmvp.select_four(full_bundle,required,pick,probe);fit=[];
if ~isempty(chosen)
    b=uavpriority.bundle(records,nav,history,macros,chosen,cfg);
    fit=uavpriority.fit_position(b,cfg);
end
end
