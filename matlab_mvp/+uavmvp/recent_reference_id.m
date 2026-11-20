function id=recent_reference_id(nav,cfg)
%RECENT_REFERENCE_ID Actual accepted-image provenance, never geometric proximity.
% This only avoids a redundant initial return acquisition; all next-leg gates
% still use the current state. Older images remain useful INS information.
id=NaN;
if ~isfield(nav,'last_reference_id') || ~isfield(nav,'last_reference_time_s'),return;end
if ~isscalar(nav.last_reference_id) || ~isfinite(nav.last_reference_id) || ...
        ~isscalar(nav.last_reference_time_s) || ~isfinite(nav.last_reference_time_s),return;end
age=nav.t-nav.last_reference_time_s;
window=cfg.navigation_candidate.reference_max_attempts_per_visit*cfg.navigation_candidate.reference_observation_s+ ...
    cfg.motion_and_budget.calculation_model_s_per_decision;
if age>=-1e-9 && age<=window+1e-9,id=nav.last_reference_id;end
end
