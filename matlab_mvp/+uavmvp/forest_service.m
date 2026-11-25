function [s,out]=forest_service(action,in,c)
%FOREST_SERVICE L02/Q01/V02/P02: actual pair book -> fit and service eligibility.
s=in.state;k=find([s.protocol.nodes.id]==string(in.peer));assert(isscalar(k));
if ~isfield(s,'fits'),s.fits=cell(numel(s.protocol.nodes),1);end
if ~isfield(s,'fit_groups'),s.fit_groups=cell(numel(s.protocol.nodes),1);end
n=s.protocol.nodes(k);records=records_for(s,in.peer);out=struct('accepted',false,'reason','');
switch action
    case 'refresh'
        if n.verified,out.reason='published_position_frozen';return,end
        s=reduce(s,struct('type','fit','id',n.id,'version',s.navigation.version, ...
            'complete_groups',0,'region_available',false,'quality',false),c);
        s.fits{k}=[];
        if isempty(records),out.reason='no_received_pairs';return,end
        try
            [input,groups]=fit_input(s,n,records,c);s.fit_groups{k}=groups;input=review_mode(input,in);
            f=uavmvp.forest_localization('fit',input,c);
        catch problem
            [s,out]=failed_fit(s,k,problem);return
        end
        groups=unique(input.fit_group_ids);complete=0;
        for j=groups(:)',complete=complete+(sum(input.fit_group_ids==j)==c.service.pairs_per_unknown_group);end
        s.fits{k}=f;s=reduce(s,struct('type','fit','id',n.id,'version',s.navigation.version, ...
            'complete_groups',complete,'region_available',true,'quality',f.quality),c);
        out=struct('accepted',true,'quality',f.quality,'complete_groups',complete,'reason','recomputed_from_received_pairs');
    case 'request_fifth'
        assert(~n.verified,'forest:Stopped','Published node cannot request more ranging.');
        f=s.fits{k};assert(~isempty(f) && f.version==s.navigation.version,'forest:Snapshot','Refresh actual four-point data first.');
        s=reduce(s,struct('type','request_fifth','id',n.id,'fit_ids',f.fit_raw_ids),c);
        out.accepted=true;
    case 'verify_group'
        assert(~isempty(n.fifth_job),'forest:HeldoutID','No issued fifth service.');
        idx=find([records.group]==in.group);assert(numel(idx)==c.service.pairs_per_unknown_group,'forest:Group','Fifth group incomplete.');
        input=fit_input(s,n,records,c);input.fifth_raw_ids=string({records(idx).id})';
        input.fifth_clone_ids=vertcat(records(idx).clone_ids);input=review_mode(input,in);
        try
            % Only F values enter preparation. Fifth measurements are read below.
            f=uavmvp.forest_localization('fit',input,c);
            s=reduce(s,struct('type','fit','id',n.id,'version',s.navigation.version,'complete_groups',4,'region_available',true,'quality',f.quality),c);
            assert(f.quality,'forest:VerificationPrerequisite','Updated four-point quality failed.');
            s=reduce(s,struct('type','reserve_verification','id',n.id,'fit_ids',f.fit_raw_ids,'fifth_ids',f.fifth_raw_ids,'snapshot',f),c);
            s=reduce(s,struct('type','verification','id',n.id,'values_m',[records(idx).range_m]','raw_ids',f.fifth_raw_ids),c);
            s.fits{k}=f;out.accepted=s.protocol.nodes(k).verified;out.reason='single_fifth_decision';
        catch problem
            if ~isempty(s.protocol.nodes(k).pending_round)
                s=reduce(s,struct('type','verification_timeout','id',n.id),c);
            elseif ~isempty(s.protocol.nodes(k).fifth_job)
                s=reduce(s,struct('type','fifth_acquisition_timeout','id',n.id),c);
            end
            [s,out]=failed_fit(s,k,problem);
        end
    case 'opportunity'
        out=opportunity(s,k,records,in.belief,c);
    case 'reselect'
        s=reduce(s,struct('type','fit_reselection','id',n.id),c);s.fit_groups{k}=[];s.fits{k}=[];
        in.state=s;[s,out]=uavmvp.forest_service('refresh',in,c);
    otherwise,error('forest:Action','Unknown service action.');
end
end

function records=records_for(s,peer)
records=[];
for j=1:numel(s.pairs)
    p=s.pairs{j};
    if p.peer==peer && strcmp(p.role,'unknown') && strcmp(p.status,'received')
        if isempty(records),records=p;else,records(end+1)=p;end %#ok<AGROW>
    end
end
end

function [input,chosenGroups]=fit_input(s,n,records,c)
ids=string({records.id})';
if ~isempty(n.pending_round)
    chosen=n.pending_round.snapshot.fit_raw_ids;
elseif ~isempty(n.fifth_job)
    chosen=n.fifth_job.fit_ids;
else
    groups=unique([records.group],'stable');complete=[];partial=[];
    for g=groups
        rows=find([records.group]==g);
        assert(numel(rows)<=c.service.pairs_per_unknown_group,'forest:Group','Too many received pairs in spatial group.');
        if numel(rows)==c.service.pairs_per_unknown_group,complete(end+1)=g;else,partial(end+1)=g;end %#ok<AGROW>
    end
    % Bounded deterministic reselection independent of range residual values.
    if numel(complete)>=4
        order=circshift(complete,-mod(n.fit_reselections,numel(complete)));order=order(1:4);
    else
        order=[complete partial];order=order(1:min(4,numel(order)));
    end
    member=[s.protocol.nodes.id]==n.id;assert(nnz(member)==1);previous=s.fit_groups{member};
    if ~isempty(previous)
        order=[previous,setdiff(order,previous,'stable')];order=order(1:min(4,numel(order)));
    end
    selected=ismember([records.group],order);chosen=ids(selected);
end
[found,k]=ismember(chosen,ids);assert(all(found),'forest:Snapshot','Frozen F pair missing from actual book.');
chosenGroups=unique([records(k).group],'stable');
input=struct('navigation',s.navigation,'fit_values_m',[records(k).range_m]', ...
    'fit_raw_ids',ids(k),'fit_group_ids',[records(k).group]', ...
    'fit_clone_ids',vertcat(records(k).clone_ids),'data_delivered',n.data_delivered);
end

function out=opportunity(s,k,records,b,c)
n=s.protocol.nodes(k);phase=uavmvp.forest_protocol('phase',n,c);
assert(b.version==s.navigation.version && strcmp(b.model_id,c.model_id) && b.t>=s.navigation.t,'forest:Snapshot','Stale candidate belief.');
[point,J]=uavmvp.forest_navigation('antenna',b,c);nav=uavmvp.forest_navigation('gate',b,c);
f=s.fits{k};
if isempty(f) || f.version~=s.navigation.version
    h=c.algorithm.grid_domain_half_width_m;region=[-h -h;-h h;h -h;h h];
else
    region=f.grid(f.active,:);
end
target=[region,repmat(c.scene.terminal_antenna_plane_z_m+b.survey(11),size(region,1),1)];
distance=max(vecnorm(target-point',2,2))+c.algorithm.spatial_sigma_multiplier*sqrt(max(eig(J*b.P*J')));
kind='range';if strcmp(phase,'DATA'),kind='data';elseif strcmp(phase,'NOTICE'),kind='control';end
probability=uavmvp.forest_radio('link',struct('distance_m',distance,'kind',kind),c);
eligible=nav.ok && all(abs(b.p(1:2))<=c.scene.allowed_corridor_half_width_m) && probability>0;
missing=c.service.pairs_per_unknown_group;partial_group=[];new_position=true;
if ismember(phase,{'INITIAL_FIT','IMPROVE','VERIFY'}) && ~isempty(records)
    % Only selected F records define new-position requirements during verification.
    selected=records;
    if ~isempty(f) && f.version==s.navigation.version
        selected=records(ismember(string({records.id}),f.fit_raw_ids));
    end
    if strcmp(phase,'VERIFY')
        assert(~isempty(f) && f.version==s.navigation.version,'forest:Snapshot','Current Q requires current F cache.');
    end
    groups=unique([selected.group],'stable');
    for group=groups
        rows=selected([selected.group]==group);clones=reshape(vertcat(rows.clone_ids)',[],1);
        [exists,ci]=ismember(clones,b.clone_ids);assert(all(exists),'forest:CloneID','Candidate lacks actual selected history.');
        same=all(vecnorm(b.clones(:,ci)-point,2,1)<=c.algorithm.group_motion_radius_m);
        separate=true;
        for a=ci(:)'
            ix=37+3*a-2+(0:2);cross=J*b.P(:,ix);
            P=J*b.P*J'+b.P(ix,ix)-cross-cross';
            margin=norm(point(1:2)-b.clones(1:2,a))-c.algorithm.spatial_sigma_multiplier*sqrt(max(0,max(eig((P(1:2,1:2)+P(1:2,1:2)')/2))));
            separate=separate && margin>=c.algorithm.spatial_separation_m;
        end
        if ~strcmp(phase,'VERIFY') && numel(rows)<c.service.pairs_per_unknown_group && same
            partial_group=group;missing=c.service.pairs_per_unknown_group-numel(rows);
        else
            new_position=new_position && separate;
        end
    end
    eligible=eligible && new_position;
end
if strcmp(phase,'VERIFY')
    eligible=eligible && n.data_delivered && n.quality && n.quality_version==s.navigation.version;
end
eligible=eligible && ~ismember(phase,{'NONE','UNSEEN'});
out=struct('id',n.id,'phase',phase,'eligible',eligible,'payload_bytes',max(0,n.expected_data_bytes-n.delivered_bytes), ...
    'missing_pairs',missing,'partial_group',partial_group,'needs_recontact',n.lease_until_s<b.t, ...
    'nominal_link_probability',probability,'candidate_antenna_m',point,'version',b.version);
end

function input=review_mode(input,in)
if isfield(in,'execution_kind')
    assert(strcmp(in.execution_kind,'fixed_input_review'));input.execution_kind=in.execution_kind;
end
end
function s=reduce(s,e,c)
s.protocol=uavmvp.forest_protocol('event',struct('state',s.protocol,'event',e),c);
end
function [s,out]=failed_fit(s,k,problem)
% Fail closed, retain raw records and an inspectable failure; no old quality reuse.
s.protocol.nodes(k).quality=false;s.protocol.nodes(k).failure_reason=problem.identifier;s.fits{k}=[];
out=struct('accepted',false,'reason',problem.identifier);
end
