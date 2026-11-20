function gate=navigation_gate(nav,duration,cfg)
nav=uavmvp.nav_prune(nav,[]); % forecast needs only the exact UAV/map marginal
gate=uavstage.navigation_gate(nav,duration,cfg);
gate.stochastic_radius_m=gate.predicted_radius_m;
gate.datum_bound_m=cfg.priority.datum_translation_bound_m;
gate.predicted_radius_m=gate.stochastic_radius_m+gate.datum_bound_m;
gate.accepted=gate.accepted && gate.predicted_radius_m<=cfg.stage.navigation_radius_limit_m;
end
