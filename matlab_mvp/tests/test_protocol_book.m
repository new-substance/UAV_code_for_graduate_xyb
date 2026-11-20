function tests=test_protocol_book
tests=functiontests(localfunctions);
end
function testWakeLostReplyPersistsWithoutInventingIdentity(t)
cfg=uavmvp.config();book=uavmvp.ProtocolBook(cfg,5,17);device=uavmvp.TerminalDevice(1,17);
control=book.group_control([1,2,3],0);packet=device.receive_group(control);
verifyFalse(t,book.nodes(1).identity_seen);verifyFalse(t,book.nodes(1).discovered);
verifyNotEmpty(t,packet);request=struct('mission_id',17,'address',1);
verifyTrue(t,device.can_respond_range(request)); % reply lost, device stays awake
timing=uavmvp.radio_timing(cfg);time=control.frame_s+packet.reply_offset_s+timing.frames.identity;
early=book.receive_identity(packet,time-.01);verifyEqual(t,early.reason,'PACKET_TIME_WINDOW');
report=book.receive_identity(packet,time);verifyTrue(t,report.accepted);
verifyTrue(t,book.nodes(1).identity_seen);verifyFalse(t,book.nodes(1).discovered);
duplicate=book.receive_identity(packet,time);verifyEqual(t,duplicate.reason,'DUPLICATE');
end
function testSessionTimeQualityAndColdTwoExchangesOneRecord(t)
[book,~,cfg]=contact();request=book.begin_ranging(1,1,91,'cold');
early=book.receive_range(rangePacket(request),1.1);verifyEqual(t,early.reason,'PACKET_TIME_WINDOW');
packet=rangePacket(request);packet.exchange_valid=[true,false];
bad=book.receive_range(packet,1.4);verifyFalse(t,bad.accepted);verifyFalse(t,book.nodes(1).discovered);
packet.exchange_valid=[true,true];packet.reported_std_m=NaN;
bad=book.receive_range(packet,1.4);verifyEqual(t,bad.reason,'RANGE_QUALITY');
repeat=book.receive_range(packet,1.4);verifyEqual(t,repeat.reason,'DUPLICATE');
request=book.begin_ranging(1,2,91,'cold');packet=rangePacket(request);packet.nav_snapshot_id=99;
bad=book.receive_range(packet,2.4);verifyEqual(t,bad.reason,'INVALID_EXCHANGE_PAIR');
packet.nav_snapshot_id=91;good=book.receive_range(packet,2.4);
verifyTrue(t,good.first_discovery);verifyEqual(t,numel(book.nodes(1).range_records),1);
verifyEqual(t,book.nodes(1).range_records{1}.nav_snapshot_id,91);
duplicate=book.receive_range(packet,2.4);verifyEqual(t,duplicate.reason,'DUPLICATE');
verifyError(t,@()book.begin_ranging(1,12,92,'warm'),'uavmvp:contactExpired');
verifyEqual(t,request.reserved_s,uavmvp.radio_timing(cfg).cold_max_s,'AbsTol',1e-12);
end
function testLateWrongTaskAndUnknownSessionCannotMutate(t)
[book,~,~]=contact();request=book.begin_ranging(1,1,91,'cold');packet=rangePacket(request);
late=book.receive_range(packet,4);verifyEqual(t,late.reason,'PACKET_TIME_WINDOW');
packet.mission_id=18;wrong=book.receive_range(packet,1.4);verifyFalse(t,wrong.accepted);
packet.mission_id=17;packet.session_id=999;wrong=book.receive_range(packet,1.4);
verifyEqual(t,wrong.reason,'UNKNOWN_SESSION');verifyFalse(t,book.nodes(1).discovered);
verifyEmpty(t,book.nodes(1).range_records);
end
function testCompletedExclusionAndPhysicalStopAreSeparate(t)
[book,device,~]=contact();request=book.begin_ranging(1,1,91,'cold');
book.receive_range(rangePacket(request),1.4);
evidence=struct('passed',true,'fit_ids',[1,2,3,4],'heldout_id',5,'nav_version',7);
verifyError(t,@()book.complete(1,evidence),'uavmvp:missingEvidence');
for k=2:5
    additional=book.begin_ranging(1,k,90+k,'warm');book.receive_range(rangePacket(additional),k+.2);
end
book.complete(1,evidence);
verifyError(t,@()book.begin_ranging(1,6,96,'cold'),'uavmvp:completedRanging');
control=book.group_control([1,2,3],6);verifyFalse(t,ismember(1,control.active_members));
verifyTrue(t,device.can_respond_range(request)); % STOP not delivered yet
book.stop_request(1,6); % lost command, consumes an attempt
command=book.stop_request(1,7);ack=device.receive_stop(command);
verifyFalse(t,device.can_respond_range(request));verifyFalse(t,book.nodes(1).stop_ack);
good=book.receive_stop_ack(ack,7.1);verifyTrue(t,good.accepted);
verifyNotEmpty(t,device.receive_stop(command)); % repeated STOP remains idempotent
other=uavmvp.ProtocolBook(uavmvp.config(),3,17);wake=other.group_control([1,2,3],5);
verifyEmpty(t,device.receive_group(wake)); % same mission cannot restart completed device
book.stop_request(1,8);
verifyError(t,@()book.stop_request(1,9),'uavmvp:stopAttemptLimit');
end
function [book,device,cfg]=contact()
cfg=uavmvp.config();book=uavmvp.ProtocolBook(cfg,5,17);device=uavmvp.TerminalDevice(1,17);
control=book.group_control([1,2,3],0);packet=device.receive_group(control);
timing=uavmvp.radio_timing(cfg);book.receive_identity(packet,control.frame_s+packet.reply_offset_s+timing.frames.identity);
end
function packet=rangePacket(request)
packet=struct('mission_id',request.mission_id,'address',request.address,'kind','RANGE_META', ...
    'session_id',request.session_id,'sequence',1,'crc_valid',true,'range_m',30, ...
    'reported_std_m',5,'exchange_valid',true(1,request.exchange_count), ...
    'exchange_sequences',request.sequences,'nav_snapshot_id',request.nav_snapshot_id, ...
    'measurement_epoch_s',request.measurement_epoch_s);
end
