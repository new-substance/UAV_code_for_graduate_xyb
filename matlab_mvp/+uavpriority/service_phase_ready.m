function [ready,unseeded]=service_phase_ready(nodes,states)
%SERVICE_PHASE_READY P02 all discovered IDs, no global four-fit barrier.
assert(numel(nodes)==numel(states));unseeded=[];
for id=1:numel(nodes)
    s=states{id};
    if ~nodes(id).discovered || (~nodes(id).completed && ...
            (isempty(s) || isempty(s.fit) || numel(s.fit.x)~=2 || any(~isfinite(s.fit.x))))
        unseeded(end+1)=id; %#ok<AGROW>
    end
end
ready=all([nodes.discovered]);
end
