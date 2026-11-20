function [owner,choice,alternatives]=select_service_owner(choices,rule)
%SELECT_SERVICE_OWNER P02 compare all current proposals; never physical truth.
assert(ismember(rule,{'stage','cost'}));owner=0;choice=[];alternatives=[];keys=[];
for id=1:numel(choices)
    c=choices{id};if isempty(c) || ~c.available,continue;end
    phase=double(~strcmp(c.stage,'VERIFY'));
    if strcmp(rule,'cost'),score=[phase,0,0,c.cost_proxy_s,id,c.candidate_id];
    elseif phase==0,score=[phase,c.range_risk_m,0,c.cost_proxy_s,id,c.candidate_id];
    else,score=[phase,-c.geometry_gain,c.range_risk_m,c.cost_proxy_s,id,c.candidate_id];end
    keys(end+1,:)=score; %#ok<AGROW>
    alternatives(end+1,:)=[id,phase,c.candidate_id,c.reference_id,c.cost_proxy_s,c.range_risk_m,c.geometry_gain]; %#ok<AGROW>
end
if isempty(keys),return;end
[~,order]=sortrows(keys);owner=keys(order(1),5);choice=choices{owner};
end
