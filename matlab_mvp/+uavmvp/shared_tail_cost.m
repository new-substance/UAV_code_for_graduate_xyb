function out=shared_tail_cost(need,probability,durations,positions,chosen,cfg,followup_cap)
%SHARED_TAIL_COST Finite fractional set-cover proxy for shared remaining visits.
% It is not a prediction of successful validation or a feasibility certificate.
if nargin<7,followup_cap=cfg.planner.tail_step_cap;end
assert(isscalar(followup_cap) && isfinite(followup_cap) && followup_cap>=0 && followup_cap==floor(followup_cap));
step_cap=min(cfg.planner.tail_step_cap,followup_cap);
remaining=max(0,need-probability(chosen,:));available=true(size(durations));available(chosen)=false;
positions_used=positions(chosen,:);cost=0;sequence=[];
for step=1:step_cap
    gains=sum(min(probability,repmat(remaining,size(probability,1),1)),2);
    for j=find(available)'
        if any(vecnorm(positions_used-positions(j,:),2,2)<cfg.localization.distinct_horizontal_separation_m)
            available(j)=false;
        end
    end
    rate=gains./max(durations,1e-6);rate(~available)=-Inf;
    [value,index]=max(rate);
    if ~isfinite(value) || value<=1e-8 || all(remaining<1e-6),break;end
    cost=cost+durations(index);sequence(end+1)=index; %#ok<AGROW>
    remaining=max(0,remaining-probability(index,:));available(index)=false;
    positions_used=[positions_used;positions(index,:)]; %#ok<AGROW>
end
% Unserved demand remains visible, rather than being silently declared done.
best=max(probability,[],1);penalty=sum(remaining./max(best,.05))*max(durations);
out=struct('time_proxy_s',cost+penalty,'scheduled_proxy_s',cost,'unresolved',remaining, ...
    'unresolved_penalty_s',penalty,'sequence',sequence,'followup_cap',step_cap);
end
