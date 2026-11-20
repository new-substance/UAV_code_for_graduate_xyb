function result=two_step_information(tracker,book,histories,beliefs,nav,first_id,second_id,cfg,first_eligible)
%TWO_STEP_INFORMATION Outcome-weighted joint information for two forecast epochs.
% nav contains both correlated antenna clones, with no fictional visual update.
% Failed first service uses the unmodified radio history for the second one.
count=numel(beliefs);values=zeros(1,count);first_values=zeros(1,count);modes=cell(count,3);
if nargin<9,first_eligible=[];else,assert(numel(first_eligible)==count);end
[both,Pboth]=uavmvp.nav_archive(nav,[first_id,second_id]);
pair_distinct=uavmvp.distinct_positions(both,Pboth,cfg);
first_time=nav.clone_times(nav.clone_ids==first_id);
second_time=nav.clone_times(nav.clone_ids==second_id);
assert(second_time>first_time);
for id=1:count
    b=beliefs{id};if isempty(b) || ~book.nodes(id).discovered,continue;end
    ids=tracker.terminals(id).pool_ids;old=[];radio_ids=[];
    if ~isempty(ids)
        records=book.nodes(id).range_records;selected=[records{ids}];
        old=[selected.effective_nav_snapshot_id];radio_ids=[selected.radio_history_id];
    end
    m=numel(old);[anchors,P]=uavmvp.nav_archive(nav,[old,first_id,second_id]);
    eligible=false(1,2);
    for a=1:2
        eligible(a)=uavmvp.distinct_positions(anchors,P,cfg,[(1:m)',repmat(m+a,m,1)]);
    end
    if ~isempty(first_eligible),eligible(1)=first_eligible(id);end
    if ~any(eligible),continue;end
    [h1,r1,mode1]=uavmvp.predict_radio_record(histories{id},first_time,cfg);
    [h11,r11,mode11]=uavmvp.predict_radio_record(h1,second_time,cfg);
    [h01,r01,mode01]=uavmvp.predict_radio_record(histories{id},second_time,cfg);
    modes(id,:)={mode1,mode11,mode01};
    C1=uavmvp.radio_history_covariance(h1,[radio_ids,r1]);
    C11=uavmvp.radio_history_covariance(h11,[radio_ids,r1,r11]);
    C01=uavmvp.radio_history_covariance(h01,[radio_ids,r01]);
    ix1=1:3*(m+1);ix2=[1:3*m,3*(m+1)+(1:3)];cols2=[1:m,m+2];
    pr=cfg.ranging.baseline_stage_probabilities;
    transport=@(mode)pr.wake_decode*pr.identity_decode_given_control*pr.range_exchange_valid^(1+strcmp(mode,'cold'));
    K=size(b.samples,1);
    for k=1:K
        x=b.samples(k,:)';in_range=vecnorm(both-[x;cfg.scene.terminal_height_m])<=cfg.scene.design_service_slant_m;
        p1=double(eligible(1) && in_range(1))*transport(mode1);
        p01=double(eligible(2) && in_range(2))*transport(mode01);
        % A successful heldout could finish the terminal: never credit another
        % RTT in that branch. This cap does not predict actual verification.
        p11=double(eligible(2) && in_range(2) && pair_distinct && ~strcmp(b.phase,'VERIFY'))*transport(mode11);
        g1=0;g01=0;g11=0;
        if p1>0
            one=uavmvp.block_information(x,anchors(:,1:m+1),P(ix1,ix1),C1,m,cfg);g1=one.information_gain;
        end
        if (1-p1)*p01>0
            only_second=uavmvp.block_information(x,anchors(:,cols2),P(ix2,ix2),C01,m,cfg);g01=only_second.information_gain;
        end
        if p1*p11>0
            joint=uavmvp.block_information(x,anchors,P,C11,m,cfg);g11=joint.information_gain;
        end
        values(id)=values(id)+(p1*(1-p11)*g1+p1*p11*g11+(1-p1)*p01*g01)/K;
        first_values(id)=first_values(id)+p1*g1/K;
    end
end
assert(all(values-first_values>=-1e-9),'uavmvp:negativeInformation','Conditional future information cannot be negative.');
result=struct('total',sum(values),'per_terminal',values,'first_total',sum(first_values), ...
    'additional_total',sum(max(0,values-first_values)),'pair_distinct',pair_distinct, ...
    'modes',{modes},'first_epoch_s',first_time,'second_epoch_s',second_time);
end
