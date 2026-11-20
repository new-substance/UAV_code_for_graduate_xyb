function choice=screen_reference_choices(nav,current,rows,cfg,deadline)
%SCREEN_REFERENCE_CHOICES Rank useful stations after the executor's first gate.
% Same-first-leg paths share one prediction. A self visit has no transit;
% its primary/local service must still pass the existing service checks.
choice=struct('accepted',false,'selected',[],'candidates',rows,'reason','NO_USEFUL_REFERENCE');
if isempty(rows),return;end
cache=cell(size(cfg.scene.reference_centers_xy_m,1),1);
for k=1:numel(rows)
    path=rows(k).path;
    assert(path.available && ~isempty(path.ids) && path.ids(1)==current, ...
        'uavmvp:referenceChoicePath','Expected an available path from the current reference.');
    if numel(path.ids)==1
        rows(k).first_leg=struct('accepted',true,'reason','NO_TRANSIT_SERVICE_UNCHECKED', ...
            'next_reference_id',current);
    else
        next=path.ids(2);
        if isempty(cache{next}),cache{next}=uavmvp.reference_transit_gate(nav,next,cfg,deadline);end
        rows(k).first_leg=cache{next};
    end
    rows(k).admissible=rows(k).first_leg.accepted;
end
choice.candidates=rows;choice.reason='NO_FEASIBLE_REFERENCE';
eligible=find([rows.admissible]);if isempty(eligible),return;end
[~,j]=max([rows(eligible).score]);choice.selected=rows(eligible(j));
choice.accepted=true;choice.reason='SERVICE_REFERENCE_SELECTED';
end
