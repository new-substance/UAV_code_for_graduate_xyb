function needed=supplemental_service_needed(states,nodes,time,cfg,mode)
%SUPPLEMENTAL_SERVICE_NEEDED Guard for entering candidate evaluation.
% During coverage retain the fixed-route protection. Once all are found,
% online geometry/probability checks decide whether a new point is useful.
if nargin<5,mode='route';end
assert(ismember(mode,{'route','online'}));
needed=false;
if strcmp(mode,'online') && all([nodes.discovered])
    needed=any(~ismember({states.phase},{'DONE','FAILED'}));
    return;
end
for id=1:numel(states)
    if ~ismember(states(id).phase,{'DONE','FAILED'}) && numel(states(id).pool_ids)>=2 && ...
            time-nodes(id).last_contact_s<=cfg.css.contact_validity_s
        needed=true;return;
    end
end
end
