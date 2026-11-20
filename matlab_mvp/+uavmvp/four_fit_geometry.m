function score=four_fit_geometry(x,anchors,Pnav,Cradio,cfg)
%FOUR_FIT_GEOMETRY Best four-row mean-information proxy before/after last row.
% The last row is prospective. After-groups must contain it. No ranges,
% residual minimization, global region certificate or acceptance decision.
m=size(anchors,2);assert(m>=5);
[~,H,C]=uavmvp.range_model(x,anchors,Pnav,Cradio,cfg.scene.terminal_height_m);
pairs=nchoosek(1:m,2);[~,lower]=uavmvp.distinct_positions(anchors,Pnav,cfg,pairs);
allowed=true(m);valid=lower>=cfg.localization.distinct_horizontal_separation_m;
for k=1:size(pairs,1)
    allowed(pairs(k,1),pairs(k,2))=valid(k);allowed(pairs(k,2),pairs(k,1))=valid(k);
end
before=best(nchoosek(1:m-1,4));
after=best([nchoosek(1:m-1,3),repmat(m,nchoosek(m-1,3),1)]);
score=struct('before',before,'after',after,'radius_reduction',0);
if isfinite(before.radius) && isfinite(after.radius)
    score.radius_reduction=max(0,before.radius-after.radius);
end
    function result=best(groups)
        result=struct('radius',Inf,'covariance',nan(2),'ids',[]);
        best_score=[Inf,Inf];
        for j=1:size(groups,1)
            ids=groups(j,:);if ~all(allowed(ids,ids),'all'),continue;end
            [L,flag]=chol(C(ids,ids),'lower');if flag,continue;end
            Z=L\H(ids,:);I=Z'*Z;if rcond(I)<1e-12,continue;end
            P=I\eye(2);value=[max(eig(P)),cond(I)];
            if value(1)<best_score(1)-1e-12 || ...
                    (abs(value(1)-best_score(1))<=1e-12 && value(2)<best_score(2))
                best_score=value;result.ids=ids;result.covariance=P;
                result.radius=sqrt(cfg.localization.region_chi2_df2_level*value(1));
            end
        end
    end
end
