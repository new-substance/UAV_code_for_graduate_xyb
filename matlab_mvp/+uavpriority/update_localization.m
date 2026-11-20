function [s,detail]=update_localization(s,b,cfg)
%UPDATE_LOCALIZATION L02/Q01/V01 per-terminal state; decoded macro bundle only.
% Exactly four fit records, followed by a later distinct heldout. No world,
% protocol side effects, physical coordinates, future samples or extra model.
if isempty(s)
    s=struct('phase','LOCALIZE','pool_ids',[],'seen_id',0,'fit',[], ...
        'fit_ids',[],'fit_version',-1,'verification_rounds',0,'proof',[]);
end
assert(~ismember(s.phase,{'DONE','FAILED'}));
members=[b.constituent_ids{:}];
assert(numel(unique(members))==numel(members),'uavproject:RepeatedConstituent', ...
    'Fit and heldout spatial records must not reuse raw services.');
id=b.record_ids(end);assert(id>s.seen_id);s.seen_id=id;
detail=struct('reason','NEED_FOUR_POSITIONS','fit',[],'check',[],'input',[]);
if ~isempty(s.pool_ids)
    [ok,rows]=ismember([s.pool_ids,id],b.record_ids);assert(all(ok));
    data=uavpriority.subbundle(b,rows);n=numel(s.pool_ids);
    if ~uavmvp.distinct_positions(data.anchors,data.Pnav,cfg,[(1:n)',(n+1)*ones(n,1)])
        detail.reason='NOT_A_NEW_POSITION';return;
    end
end
s.pool_ids=[s.pool_ids,id];
if strcmp(s.phase,'VERIFY')
    [ok,rows]=ismember([s.fit_ids,id],b.record_ids);assert(all(ok));
    data=uavpriority.subbundle(b,rows);
    if s.fit_version~=b.version
        s.fit=uavpriority.fit_position(uavpriority.subbundle(data,1:4),cfg);
        s.fit_version=b.version;
    end
    check=uavpriority.verify_position(s.fit,data,cfg);s.verification_rounds=s.verification_rounds+1;
    detail.fit=uavstage.compact_fit(s.fit);detail.check=check;detail.input=compact(data);
    detail.reason=check.reason;
    if check.passed
        s.phase='DONE';s.proof=struct('passed',true,'fit_ids',s.fit_ids, ...
            'heldout_id',id,'nav_version',b.version,'check',check);return;
    end
    s.phase='LOCALIZE';
    if s.verification_rounds>=cfg.localization.verification_round_cap
        s.phase='FAILED';detail.reason='VERIFICATION_LIMIT';return;
    end
    % This round is over. The failed heldout is eligible only in a NEW fit.
elseif numel(s.pool_ids)<4
    return;
end
[ok,rows]=ismember(s.pool_ids,b.record_ids);assert(all(ok));
pool=uavpriority.subbundle(b,rows);required=[];
if numel(s.pool_ids)>4,required=id;end
probe=[cfg.scene.width_m;cfg.scene.height_m]/2;
if ~isempty(s.fit) && all(isfinite(s.fit.x)),probe=s.fit.x;end
selected=uavpriority.selection_bundle(pool,required,cfg);pick=cfg;
pick.localization.group_geometry_rule='global_geometry';
pick.localization.region_chi2_df2_level=cfg.stage.region_gamma;
[chosen,~]=uavmvp.select_four(selected,required,pick,probe);
if isempty(chosen),detail.reason='NO_DISTINCT_FOUR_GROUP';return;end
[ok,rows]=ismember(chosen,b.record_ids);assert(all(ok));data=uavpriority.subbundle(b,rows);
s.fit=uavpriority.fit_position(data,cfg);s.fit_ids=chosen;s.fit_version=b.version;
detail.reason=s.fit.reason;
% Preserve the failed-heldout evidence even when a new fit is attempted.
if isempty(detail.check)
    detail.input=compact(data);detail.fit=uavstage.compact_fit(s.fit);
else
    detail.recovery_input=compact(data);detail.recovery_fit=uavstage.compact_fit(s.fit);
end
if s.fit.quality,s.phase='VERIFY';end
if numel(s.pool_ids)>=cfg.localization.new_positions_per_terminal_cap
    s.phase='FAILED';detail.reason='POSITION_LIMIT';
end
end
function out=compact(b)
fields={'anchors','ranges','Pnav','Pnavigation','Ptiming','Cradio', ...
    'record_ids','nav_ids','version','model_bias_bounds_m','constituent_ids'};
out=struct();for k=1:numel(fields),out.(fields{k})=b.(fields{k});end
end
