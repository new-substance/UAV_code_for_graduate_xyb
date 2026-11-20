function [histories,visit]=service_stop(world,book,tracker,histories,cfg,visit_id,deadline,visit_start)
%SERVICE_STOP Common finite executor: all first contacts, all first RTTs,
% supplemental contacts/RTTs, then finite STOP. No physical-truth input.
if nargin<8,visit_start=world.t;end
finish_by=min(deadline,visit_start+cfg.motion_and_budget.stop_max_dwell_s);
count=numel(book.nodes);eligible=false(1,count);
for id=1:count,eligible(id)=uavmvp.can_add_position(tracker,book,id,world.nav,cfg);end
targets=find(eligible);responded=false(1,count);attempts=zeros(1,count);contact_counts=zeros(1,count);
timing=uavmvp.radio_timing(cfg);
visit=struct('visit_id',visit_id,'start_s',visit_start,'end_s',world.t,'targets',targets, ...
    'opportunity_complete',false,'reason','NO_TARGETS','actions',{{}},'attempts',attempts,'contact_counts',contact_counts);
if isempty(targets),send_notices();visit.end_s=world.t;return;end
groups=unique(ceil(targets/cfg.css.group_size));groups=circshift(groups,[0,-mod(visit_id-1,numel(groups))]);
mandatory=uavmvp.first_round_budget(cfg,targets);
if world.t+mandatory>finish_by+1e-10
    visit.reason='FIRST_ROUND_BUDGET_UNAVAILABLE';visit.end_s=world.t;return;
end
world.hold(cfg.motion_and_budget.settle_s);
for group=groups
    ids=targets(ceil(targets/cfg.css.group_size)==group);contact(ids,1);
end
order=circshift(targets,[0,-mod(visit_id-1,numel(targets))]);
for id=order
    if responded(id),serve(id,1);end
end
visit.opportunity_complete=all(contact_counts(targets)>=1) && all(attempts(targets(responded(targets)))>=1);
for group=groups
    ids=targets(ceil(targets/cfg.css.group_size)==group & ~responded(targets));
    if isempty(ids),continue;end
    g=uavmvp.radio_timing(cfg,numel(ids));reserve=g.group_s+cfg.css.wake_ready_s+ ...
        numel(ids)*(timing.frames.identity+cfg.css.guard_s+timing.cold_max_s+cfg.motion_and_budget.calculation_model_s_per_decision);
    if world.t+reserve>finish_by,continue;end
    contact(ids,2);
    for id=ids
        if responded(id) && attempts(id)==0,serve(id,1);end
    end
end
for id=order
    if responded(id) && attempts(id)<cfg.css.max_service_attempts_per_stop && ...
            uavmvp.can_add_position(tracker,book,id,world.nav,cfg)
        serve(id,2);
    end
end
send_notices();world.prune(tracker.needed_clones());
visit.reason='VISIT_FINISHED';visit.end_s=world.t;visit.attempts=attempts;visit.contact_counts=contact_counts;

    function contact(ids,round)
        control=book.group_control(ids,world.t);start=world.t;
        packets=world.transmit_group(control);contact_counts(ids)=contact_counts(ids)+1;
        accepted=[];
        for index=1:numel(packets)
            packet=packets{index};r=book.receive_identity(packet,packet.received_time_s);
            if r.accepted,responded(packet.address)=true;accepted(end+1)=packet.address;end %#ok<AGROW>
        end
        visit.actions{end+1}=struct('kind','CONTACT','round',round,'addresses',ids, ...
            'start_s',start,'end_s',world.t,'accepted',accepted);
    end
    function serve(id,round)
        if ~uavmvp.can_add_position(tracker,book,id,world.nav,cfg),return;end
        if world.t-book.nodes(id).last_contact_s>cfg.css.contact_validity_s,return;end
        reserve=cfg.css.exchange_deadline_s;
        history=histories{id};
        if ~isfinite(history.kappa) || world.t+timing.exchange_s+1e-5-history.calibration_time_s>cfg.ranging.clock_calibration_max_age_s
            reserve=timing.cold_max_s;
        end
        compute=cfg.motion_and_budget.calculation_model_s_per_decision;
        if world.t+reserve+compute>finish_by,return;end
        start=world.t;attempts(id)=attempts(id)+1;
        [histories{id},report,~]=uavmvp.execute_range_service(world,book,histories{id},id,cfg);
        if report.accepted,tracker.ingest(id,world.nav,histories{id});end
        world.hold(compute);
        visit.actions{end+1}=struct('kind','RTT','round',round,'address',id,'start_s',start, ...
            'end_s',world.t,'accepted',report.accepted,'reason',report.reason);
    end
    function send_notices()
        for address=1:count
            node=book.nodes(address);
            if ~node.completed || node.stop_ack || node.stop_attempts>=cfg.planner.stop_notice_attempt_cap,continue;end
            reserve=max(timing.stop_success_s,cfg.css.exchange_deadline_s);
            if world.t+reserve>finish_by,continue;end
            start=world.t;command=book.stop_request(address,start);ack=world.transmit_stop(command);
            accepted=false;
            if ~isempty(ack),result=book.receive_stop_ack(ack,ack.received_time_s);accepted=result.accepted;end
            visit.actions{end+1}=struct('kind','STOP','address',address,'start_s',start,'end_s',world.t,'accepted',accepted);
        end
    end
end
