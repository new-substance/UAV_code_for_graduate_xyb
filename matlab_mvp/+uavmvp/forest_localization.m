function out=forest_localization(action,in,cfg)
%FOREST_LOCALIZATION D01/L02/Q01/V02: finite correlated grid, never nav updates.
switch action
    case 'condition'
        % L02-a. Legal external Gaussian position prior; rank/SPD is required.
        P=uavmvp.forest_covariance(in.joint_covariance);nx=numel(in.x_mean);
        Px=P(1:nx,1:nx);B=P(nx+1:end,1:nx);L=chol(Px,'lower');
        K=(B/L')/L;
        out=struct('mean',in.nuisance_mean(:)+K*(in.x(:)-in.x_mean(:)), ...
            'covariance',uavmvp.forest_covariance(P(nx+1:end,nx+1:end)-K*B'));
    case 'fit'
        out=prepare_fit(in,cfg);
    case 'verify'
        f=in.snapshot;v=in.values_m(:);a=cfg.algorithm.predictive_alpha;
        clock=tic;
        assert(f.quality && f.data_delivered && f.version==in.version && ...
            f.spatial_valid && f.fifth_count==numel(v),'forest:VerificationPrerequisite','Invalid heldout snapshot.');
        assert(isequal(string(in.raw_ids(:)),f.fifth_raw_ids(:)) && all(isfinite(v)), ...
            'forest:HeldoutID','Heldout values must match the reserved immutable block.');
        w=cfg.algorithm.fifth_projection_weights(:);z=w'*v;
        mu=f.conditional_mean*w;s2=zeros(numel(f.cost),1);D=s2;
        for g=1:numel(s2)
            if f.runtime_deadline_enforced && toc(clock)>cfg.service.computation_bound_s
                error('forest:ComputationTimeout','Heldout calculation exceeded its wall-clock budget.');
            end
            C=f.conditional_covariance(:,:,g);L=chol(C,'lower');s2(g)=w'*C*w;
            e=L\(v-f.conditional_mean(g,:)');D(g)=e'*e;
        end
        assert(all(s2>0));cdf=.5*erfc(-(z-mu)./sqrt(2*s2));u=f.weights'*cdf;
        q=2*gammaincinv(1-a,numel(v)/2);
        compatible=f.active & cdf>=a/2 & cdf<=1-a/2 & D<=q;
        out=struct('passed',u>=a/2 && u<=1-a/2 && any(compatible), ...
            'mixed_cdf',u,'full_statistic',D,'chi_square_threshold',q, ...
            'compatible_grid_ids',find(compatible),'published_xy_m',f.map_absolute_xy_m, ...
            'version',f.version,'weights_unchanged',true,'kind','nominal_model_compatibility_not_true_accuracy');
    otherwise,error('forest:Action','Unknown localization action.');
end
end

function out=prepare_fit(in,c)
n=in.navigation;a=c.algorithm;m=c.service.pairs_per_unknown_group;
clock=tic;enforce=true;
if isfield(in,'execution_kind')
    assert(strcmp(in.execution_kind,'fixed_input_review'),'forest:ExecutionKind','Only fixed-input review can omit runtime deadlines.');
    enforce=false;
end
assert(strcmp(n.model_id,c.model_id));
nf=numel(in.fit_values_m);ids=string(in.fit_raw_ids(:));groups=in.fit_group_ids(:);
assert(nf>0 && nf<=4*m && numel(ids)==nf && numel(groups)==nf && all(isfinite(in.fit_values_m(:))));
assert(numel(unique(ids))==nf,'forest:Duplicate','Duplicate fit pair.');
hasV=isfield(in,'fifth_clone_ids') && ~isempty(in.fifth_clone_ids);
nv=0;vIds=strings(0,1);allClones=direction_ids(in.fit_clone_ids,nf);
if hasV
    nv=m;vIds=string(in.fifth_raw_ids(:));
    assert(nv==m && numel(vIds)==nv && numel(unique([ids;vIds]))==nf+nv,'forest:HeldoutID','Fit/heldout pair overlap.');
    allClones=[allClones;direction_ids(in.fifth_clone_ids,nv)];
end
flat=reshape(allClones',[],1);
[exists,k]=ismember(flat,n.clone_ids);assert(all(exists),'forest:CloneID','Missing antenna-position history.');
assert(all(n.clone_times(k)<=n.t),'forest:Epoch','Future unknown measurement pose.');
times=reshape(n.clone_times(k),2,[])';
assert(all(abs(times(:,1)-times(:,2))<=c.radio.pair_whole_attempt_bound_s),'forest:Epoch','Pair support exceeds attempt deadline.');
poses=n.clones(:,k);rows=nf+nv;uniqueGroups=unique(groups,'stable');complete=numel(uniqueGroups)==4;
spatial=true;
if hasV
    rv=2*nf+(1:2*nv);
    for x=rv
        for y=rv
            assert(norm(poses(:,x)-poses(:,y))<=c.algorithm.group_motion_radius_m,'forest:GroupMotion','Fifth block moved beyond group tolerance.');
        end
    end
end
for i=1:numel(uniqueGroups)
    pairs=find(groups==uniqueGroups(i));complete=complete && numel(pairs)==m;
    assert(numel(pairs)<=m,'forest:Group','Too many pairs in one spatial group.');
    r=reshape([2*pairs'-1;2*pairs'],1,[]);
    for x=r(:)'
        for y=r(:)'
            assert(norm(poses(:,x)-poses(:,y))<=c.algorithm.group_motion_radius_m,'forest:GroupMotion','Group moved beyond registered tolerance.');
        end
    end
    for j=1:i-1
        pj=find(groups==uniqueGroups(j));rj=reshape([2*pj'-1;2*pj'],1,[]);spatial=spatial && separated(n,k,r,rj,c);
    end
    if hasV,spatial=spatial && separated(n,k,r,2*nf+(1:2*nv),c);end
end
half=a.grid_domain_half_width_m;pitch=a.grid_pitch_m;axis=-half:pitch:half;
assert(abs(axis(end)-half)<1e-10 && pitch>0);
% ndgrid ordering is explicitly sorted (x,y) for deterministic tie breaking.
[xx,yy]=ndgrid(axis,axis);grid=sortrows([xx(:),yy(:)],[1 2]);ng=size(grid,1);
assert(ng<=c.finite_policy_limits.max_grid_points,'forest:GridBudget','Grid exceeds point budget.');
width=pitch*ones(numel(axis),1);width([1 end])=pitch/2;
[~,ix]=ismember(grid(:,1),axis);[~,iy]=ismember(grid(:,2),axis);
mass=width(ix).*width(iy)/(2*half)^2;
cost=Inf(ng,1);cm=zeros(ng,nv);cv=zeros(nv,nv,ng);
for g=1:ng
    if enforce && toc(clock)>c.service.computation_bound_s
        error('forest:ComputationTimeout','Grid calculation exceeded its declared wall-clock budget.');
    end
    target=[grid(g,:)';c.scene.terminal_antenna_plane_z_m+n.survey(11)];
    v=poses-target;d=sqrt(sum(v.^2,1))';assert(all(d>1e-6),'forest:Geometry','Coincident unknown/antenna.');
    u=(v./d')';J=zeros(rows,size(n.P,1));meanDistance=zeros(rows,1);
    for r=1:rows
        for direction=1:2
            j=2*r-2+direction;w=c.radio.pair_direction_weights(direction);idx=37+3*k(j)-2+(0:2);
            J(r,idx)=J(r,idx)+w*u(j,:);J(r,26)=J(r,26)-w*u(j,3);
            meanDistance(r)=meanDistance(r)+w*d(j);
        end
    end
    J(:,29)=1;
    % Clone already contains antenna attitude/lever error: do not add it twice.
    C=J*n.P*J'+c.radio.unknown_pair_white_std_m^2*eye(rows)+ ...
        c.radio.unknown_persistent_link_std_m^2*ones(rows);
    meanR=meanDistance+c.radio.unknown_persistent_link_mean_m+n.device_bias_m;
    CF=C(1:nf,1:nf);L=chol((CF+CF')/2,'lower');e=in.fit_values_m(:)-meanR(1:nf);z=L\e;
    cost(g)=z'*z+2*sum(log(diag(L)))-2*log(mass(g));
    if hasV
        B=C(nf+1:end,1:nf);K=(B/L')/L;
        cm(g,:)=(meanR(nf+1:end)+K*e)';
        cv(:,:,g)=uavmvp.forest_covariance(C(nf+1:end,nf+1:end)-K*B');
        chol(cv(:,:,g));
    end
end

if enforce && toc(clock)>c.service.computation_bound_s
    error('forest:ComputationTimeout','Grid calculation exceeded its declared wall-clock budget.');
end
[bestCost,best]=min(cost);active=cost-bestCost<=a.nominal_cost_increment;
weights=exp(-.5*(cost-bestCost));weights=weights/sum(weights);
relativeRadius=max(vecnorm(grid(active,:)-grid(best,:),2,2))+pitch/sqrt(2);
homeEnvelope=a.home_envelope_multiplier*sqrt(max(eig(n.P(35:36,35:36))));
radius=relativeRadius+homeEnvelope;
out=struct('version',n.version,'map_relative_xy_m',grid(best,:), ...
    'map_absolute_xy_m',grid(best,:)+n.home(1:2)','grid',grid,'cost',cost, ...
    'weights',weights,'active',active,'nominal_radius_m',radius, ...
    'relative_radius_m',relativeRadius,'home_envelope_m',homeEnvelope, ...
    'quality',complete && spatial && radius<=a.quality_radius_m, ...
    'data_delivered',in.data_delivered,'spatial_valid',spatial, ...
    'fit_raw_ids',ids,'fifth_raw_ids',vIds,'fifth_count',nv, ...
    'conditional_mean',cm,'conditional_covariance',cv,'complete_four_groups',complete, ...
    'runtime_deadline_enforced',enforce, ...
    'kind','finite_grid_nominal_cost_region_not_continuous_confidence_set');
end

function ids=direction_ids(ids,n)
ids=string(ids);
if numel(ids)==n,ids=repmat(ids(:),1,2);end
assert(isequal(size(ids),[n 2]),'forest:CloneID','Each pair needs two directional pose IDs.');
end

function yes=separated(n,k,ra,rb,c)
yes=true;
for a=ra(:)'
    for b=rb(:)'
        ia=37+3*k(a)-2+(0:1);ib=37+3*k(b)-2+(0:1);
        P=n.P(ia,ia)+n.P(ib,ib)-n.P(ia,ib)-n.P(ib,ia);
        margin=norm(n.clones(1:2,k(a))-n.clones(1:2,k(b)))- ...
            c.algorithm.spatial_sigma_multiplier*sqrt(max(0,max(eig((P+P')/2))));
        yes=yes && margin>=c.algorithm.spatial_separation_m;
    end
end
end
