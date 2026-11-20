function out=selection_bundle(b,required,cfg)
%SELECTION_BUNDLE Bounded geometric candidate pool, never measured residuals.
n=numel(b.record_ids);cap=cfg.priority.selection_pool_cap;
if n<=cap,out=b;return;end
xy=b.anchors(1:2,:);rows=[];
if ~isempty(required),[ok,k]=ismember(required,b.record_ids);assert(ok);rows=k;end
for axis=1:2
    [~,a]=min(xy(axis,:));[~,z]=max(xy(axis,:));rows=unique([rows,a,z],'stable');
end
while numel(rows)<cap
    distance=Inf(1,n);
    for row=rows,distance=min(distance,vecnorm(xy-xy(:,row),2,1));end
    distance(rows)=-Inf;[~,k]=max(distance);rows(end+1)=k; %#ok<AGROW>
end
out=uavpriority.subbundle(b,sort(rows));
end
