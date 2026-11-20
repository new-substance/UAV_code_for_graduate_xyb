function preview=reference_belief_preview(nav,id,cfg,attempts)
%REFERENCE_BELIEF_PREVIEW Conditional information for PLANNING ONLY.
% Enumerate missing/accepted image branches; no pixels, mean correction,
% navigation version increment, or real-world state update is produced.
assert(attempts>=1 && attempts<=2 && attempts==floor(attempts));
branches=struct('nav',nav,'weight',1,'hits',0);
gate_weight=gammainc(cfg.sim.reference_nis_gate/2,4);
for frame=1:attempts
    next=struct('nav',{},'weight',{},'hits',{});
    for j=1:numel(branches)
        parent=branches(j);prop=uavmvp.predict_hold(parent.nav,cfg.navigation_candidate.reference_observation_s,cfg);
        margin=uavmvp.reference_margin(prop.nav,id,cfg);p=0;
        if prop.envelope_inside_area && margin.envelope_inside
            p=cfg.navigation_candidate.marker_detect_probability_given_visibility*gate_weight;
        end
        if p<1
            next(end+1)=struct('nav',prop.nav,'weight',parent.weight*(1-p),'hits',parent.hits); %#ok<AGROW>
        end
        if p>0
            success=prop.nav;[~,H]=uavmvp.camera_model(success,id,cfg);
            V=(cfg.navigation_candidate.camera.pixel_std*cfg.sim.camera_noise_scale)^2*eye(8);
            P=success.P;K=(P*H')/(H*P*H'+V);A=eye(size(P))-K*H;
            success.P=A*P*A'+K*V*K';success.P=(success.P+success.P')/2;
            next(end+1)=struct('nav',success,'weight',parent.weight*p,'hits',parent.hits+1); %#ok<AGROW>
        end
    end
    branches=next;
end
weights=[branches.weight];hits=[branches.hits];q=sum(weights(hits>0));
assert(abs(sum(weights)-1)<1e-12);
conditional=branches(1).nav;
if q>0
    conditional.P=zeros(size(nav.P));
    for j=find(hits>0),conditional.P=conditional.P+weights(j)/q*branches(j).nav.P;end
end
preview=struct('conditional_nav',conditional,'success_probability_proxy',q, ...
    'branch_weights',weights,'branch_accepted_frames',hits, ...
    'model','Linear-Gaussian conditional error covariance; detection/gate weights approximate; never execution permission');
end
