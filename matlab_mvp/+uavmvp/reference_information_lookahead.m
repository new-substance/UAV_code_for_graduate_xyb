function rows=reference_information_lookahead(rows,forecast_navs,nav,reference_id,tracker,book,histories,beliefs,cfg,deadline,remaining,previews)
%REFERENCE_INFORMATION_LOOKAHEAD Conditional reference branches for ranking only.
% First action set/permission and all real observation/return gates are shared
% unchanged with other policies. Synthetic covariance is never executed.
attempts=cfg.navigation_candidate.reference_max_attempts_per_visit;
onsite=attempts*cfg.navigation_candidate.reference_observation_s;
calculation=cfg.motion_and_budget.calculation_model_s_per_decision;
if nargin<12,previews=cell(1,numel(rows));end
for a=1:numel(rows)
    if isempty(previews{a})
        previews{a}=uavmvp.reference_service_preview(forecast_navs{a},rows(a).conditional_on_actual_reference, ...
            rows(a).window_s,nav.t,reference_id,tracker,book,histories,beliefs,cfg,deadline);
    end
    preview=previews{a};first=preview.nav;first_value=preview.value;
    qA=preview.summary.reference_weight;admissible=preview.summary.admissible;
    first_gain=sum(preview.summary.information);single_duration=preview.summary.duration_proxy_s;
    rows(a).score=-first_gain/max(single_duration,1e-6);
    rows(a).horizon=struct('sequence',a,'gain',first_gain,'duration_proxy_s',single_duration, ...
        'evaluated_pairs',0,'feasible_pairs',0,'second_epoch_s',NaN,'first_reference_weight',qA, ...
        'continuation_reference_weight',0,'first_forecast_admissible',admissible, ...
        'model','Conditional visual-information proxy for ranking only; real execution unchanged', ...
        'screening',struct('next',{},'reason',{},'continuation_weight',{}));
    if remaining<2 || ~admissible,continue;end
    [first,first_id]=uavmvp.nav_clone(first,cfg);
    serviced=uavmvp.predict_hold(first,rows(a).window_s,cfg);
    % Match the actual on-site image pre-screen. A failed on-site branch may
    % use the already-reserved center fallback, but earns no second-step gain.
    post_gate=uavmvp.predict_hold(serviced.nav,onsite,cfg);
    margin=uavmvp.reference_margin(post_gate.nav,reference_id,cfg);
    qR=0;post=[];
    if serviced.envelope_inside_area && post_gate.envelope_inside_area && margin.envelope_inside
        post=uavmvp.reference_belief_preview(serviced.nav,reference_id,cfg,attempts);
        qR=post.success_probability_proxy;
    end
    for b=1:numel(rows)
        if a==b,continue;end
        rows(a).horizon.evaluated_pairs=rows(a).horizon.evaluated_pairs+1;
        detail=struct('next',b,'reason','POST_REFERENCE_UNAVAILABLE','continuation_weight',0);
        if qR==0,rows(a).horizon.screening(end+1)=detail;continue;end
        future=post.conditional_nav;
        % Reuse the complete one-action acquisition gate on this CONDITIONAL
        % branch. It never authorizes the real second action in advance.
        acquire=norm(rows(b).target-future.p)>.4;qB=1;
        if acquire
            pre=uavmvp.predict_service_visit(future,rows(b).target,reference_id,cfg,onsite);
            frame=uavmvp.predict_hold(pre.outward.nav,cfg.navigation_candidate.reference_observation_s,cfg);
            view=uavmvp.reference_margin(frame.nav,reference_id,cfg);
            escape=uavmvp.reference_return_plan(pre.reacquisition.nav,cfg,reference_id);
            if ~pre.accepted || ~view.envelope_inside || ~escape.available || ...
                    future.t+pre.duration_s+escape.reserve_s>deadline
                detail.reason='SECOND_ACQUISITION';rows(a).horizon.screening(end+1)=detail;continue;
            end
            before_service=uavmvp.reference_belief_preview(pre.outward.nav,reference_id,cfg,attempts);
            qB=before_service.success_probability_proxy;future=before_service.conditional_nav;
        end
        decision=uavmvp.predict_hold(future,calculation,cfg);future=decision.nav;
        finish=uavmvp.predict_service_visit(future,future.p,reference_id,cfg,rows(b).window_s,onsite);
        back=uavmvp.reference_return_plan(finish.reacquisition.nav,cfg,reference_id);
        duration=future.t-nav.t+finish.duration_s;
        if qB==0 || ~finish.accepted || ~back.available || nav.t+duration+back.reserve_s>deadline
            detail.reason='SECOND_SERVICE_RETURN';rows(a).horizon.screening(end+1)=detail;continue;
        end
        [future,second_id]=uavmvp.nav_clone(future,cfg);
        value=uavmvp.two_step_information(tracker,book,histories,beliefs,future,first_id,second_id,cfg,first_value.eligible);
        gain=first_gain+qA*qR*qB*value.additional_total;
        detail.reason='PASSED';detail.continuation_weight=qA*qR*qB;
        rows(a).horizon.screening(end+1)=detail;
        rows(a).horizon.feasible_pairs=rows(a).horizon.feasible_pairs+1;
        score=-gain/max(duration,1e-6);
        if score<rows(a).score
            rows(a).score=score;rows(a).horizon.sequence=[a,b];rows(a).horizon.gain=gain;
            rows(a).horizon.duration_proxy_s=duration;rows(a).horizon.second_epoch_s=value.second_epoch_s;
            rows(a).horizon.continuation_reference_weight=qA*qR*qB;
        end
    end
end
end
