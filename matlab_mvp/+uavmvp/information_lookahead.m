function rows=information_lookahead(rows,forecast_navs,nav,reference_id,tracker,book,histories,beliefs,cfg,deadline,remaining)
%INFORMATION_LOOKAHEAD Generic horizon <= 2 over the common local action set.
% Only the first action executes. No mean range or future reference update is
% supplied to the real tracker. Pair values are advisory open-loop proxies.
onsite=cfg.navigation_candidate.reference_max_attempts_per_visit*cfg.navigation_candidate.reference_observation_s;
calculation=cfg.motion_and_budget.calculation_model_s_per_decision;
for a=1:numel(rows)
    gain=sum(rows(a).information);duration=rows(a).duration_proxy_s;
    rows(a).score=-gain/max(duration,1e-6);
    rows(a).horizon=struct('sequence',a,'gain',gain,'duration_proxy_s',duration, ...
        'evaluated_pairs',0,'feasible_pairs',0,'second_epoch_s',NaN, ...
        'model','Open-loop mean information; actual observations required before execution', ...
        'screening',struct('next',{},'reason',{},'second_pixel_margin',{},'duration_s',{},'return_reserve_s',{}));
    if remaining<2,continue;end
    % Like the common single-step score, the first antenna proxy is at the
    % forecast arrival epoch. Acquisition/serial-radio offsets are not expanded
    % there. Charge their window before propagating to the second candidate.
    [first,first_id]=uavmvp.nav_clone(forecast_navs{a},cfg);
    acquisition=onsite*double(rows(a).conditional_on_actual_reference);
    hold=uavmvp.predict_hold(first,acquisition+calculation+rows(a).window_s+onsite,cfg);
    for b=1:numel(rows)
        if a==b,continue;end
        rows(a).horizon.evaluated_pairs=rows(a).horizon.evaluated_pairs+1;
        travel=uavmvp.predict_leg(hold.nav,rows(b).target,cfg,60);
        exposure=uavmvp.predict_hold(travel.nav,onsite+calculation,cfg);
        margin=uavmvp.reference_margin(exposure.nav,reference_id,cfg);
        detail=struct('next',b,'reason','PASSED','second_pixel_margin',margin.pixel_margin, ...
            'duration_s',NaN,'return_reserve_s',NaN);
        if ~hold.envelope_inside_area,detail.reason='FIRST_HOLD_AREA';
        elseif ~travel.arrived,detail.reason='SECOND_MOVEMENT';
        elseif ~travel.envelope_inside_area || ~exposure.envelope_inside_area,detail.reason='SECOND_AREA';
        elseif ~margin.envelope_inside,detail.reason='SECOND_REFERENCE_VIEW';end
        if ~strcmp(detail.reason,'PASSED')
            rows(a).horizon.screening(end+1)=detail;continue;
        end
        % Forecast service, possible failed on-site frames, center fallback,
        % and conditional return. No camera covariance reduction is credited.
        finish=uavmvp.predict_service_visit(exposure.nav,exposure.nav.p,reference_id,cfg,rows(b).window_s,onsite);
        back=uavmvp.reference_return_plan(finish.reacquisition.nav,cfg,reference_id);
        pair_duration=exposure.nav.t-nav.t+finish.duration_s;
        detail.duration_s=pair_duration;detail.return_reserve_s=back.reserve_s;
        if ~finish.accepted,detail.reason='SECOND_SERVICE_AND_REACQUISITION';
        elseif ~back.available,detail.reason='CONDITIONAL_RETURN';
        elseif nav.t+pair_duration+back.reserve_s>deadline,detail.reason='TOTAL_BUDGET';end
        rows(a).horizon.screening(end+1)=detail;
        if ~strcmp(detail.reason,'PASSED'),continue;end
        [future,second_id]=uavmvp.nav_clone(exposure.nav,cfg);
        value=uavmvp.two_step_information(tracker,book,histories,beliefs,future,first_id,second_id,cfg);
        rows(a).horizon.feasible_pairs=rows(a).horizon.feasible_pairs+1;
        score=-value.total/max(pair_duration,1e-6);
        if score<rows(a).score
            rows(a).score=score;rows(a).horizon.sequence=[a,b];rows(a).horizon.gain=value.total;
            rows(a).horizon.duration_proxy_s=pair_duration;rows(a).horizon.second_epoch_s=value.second_epoch_s;
        end
    end
end
end
