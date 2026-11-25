function out=forest_frame(action,in)
%FOREST_FRAME C01/C02: application bytes, network byte order; PHY CRC is separate.
% Type codes 1 WAKE,2 ID_STATUS,3 DATA,4 RANGE_CONTROL,5 RANGE_REPORT,6 ACK,7 COMPLETE.
if strcmp(action,'encode')
    h=in.header;m=in.message;body=encode_body(h.type,m);
    assert(h.version==1,'forest:FrameVersion','Unsupported protocol version.');
    out=[u(h.version,1),u(h.type,1),u(h.mission,2),u(h.session,2),u(h.seq,2), ...
        u(h.source,2),u(h.target,2),u(h.flags,1),u(numel(body),1),body];
    assert(numel(out)<=255,'forest:FrameSize','Application frame exceeds PHY payload limit.');
elseif strcmp(action,'decode')
    assert(isa(in,'uint8'),'forest:FrameField','Decoded wire input must be uint8 bytes.');
    b=uint8(in(:)');assert(numel(b)>=14 && numel(b)<=255 && numel(b)==14+double(b(14)), ...
        'forest:FrameSize','Bad application length.');
    h=struct('version',v(b(1)),'type',v(b(2)),'mission',v(b(3:4)), ...
        'session',v(b(5:6)),'seq',v(b(7:8)),'source',v(b(9:10)), ...
        'target',v(b(11:12)),'flags',v(b(13)));
    assert(h.version==1,'forest:FrameVersion','Unsupported protocol version.');
    m=decode_body(h.type,b(15:end));out=struct('header',h,'message',m);
else
    error('forest:Action','Unknown frame action.');
end
end

function b=encode_body(type,m)
switch type
    case 1
        K=numel(m.members);assert(K<=45,'forest:FrameSize','Split WAKE member lists.');
        b=[u(m.train_index,2),u(m.remaining_us,4),u(m.lease_ms,4),u(m.slot_us,4),u(K,1)];
        for k=1:K,x=m.members(k);b=[b,u(x.id,2),u(x.role,1),u(x.slot_index,2)];end %#ok<AGROW>
    case 2
        b=[u(m.capability,2),u(m.battery_mV,2),u(m.data_seq,2),u(m.data_bytes,2),u(m.sample_epoch_local,4)];
    case 3
        assert(m.fragment_count>=1 && m.fragment_index<m.fragment_count && m.fragment_index>=0);
        assert(numel(m.samples)<=231,'forest:FrameSize','Split DATA payload.');
        assert(all(isfinite(double(m.samples(:)))) && all(m.samples(:)>=0 & m.samples(:)<=255) && ...
            all(double(m.samples(:))==fix(double(m.samples(:)))),'forest:FrameField','Invalid sample bytes.');
        b=[u(m.data_seq,2),u(m.fragment_index,1),u(m.fragment_count,1),u(m.sample_epoch_local,4),u(m.sensor_flags,2),uint8(m.samples(:)')];
    case 4
        b=[u(m.transaction,2),u(m.role,1),u(m.count,1),u(m.peer,2),u(m.sf,1),u(m.bw_code,1),u(m.calibration_id,2),u(m.start_after_us,4)];
    case 5
        assert(m.raw_result==fix(m.raw_result) && m.raw_result>=-2^23 && m.raw_result<2^23,'forest:Raw','RAW must be signed-24 extended.');
        b=[u(m.transaction,2),u(mod(double(m.raw_result),2^32),4),u(m.quality_flags,2),u(m.calibration_id,2),u(m.epoch_local,4)];
    case 6,b=[u(m.acked_type,1),u(m.acked_seq,2),u(m.status,1)];
    case 7,b=[u(m.result_seq,2),u(m.action,1),u(m.ack_window_us,4)];
    otherwise,error('forest:FrameType','Unknown application type.');
end
end

function m=decode_body(type,b)
sizes=[15 12 10 14 14 4 7];assert(type>=1 && type<=7,'forest:FrameType','Unknown application type.');
assert(numel(b)>=sizes(type),'forest:FrameSize','Truncated body.');
switch type
    case 1
        K=v(b(15));assert(K<=45 && numel(b)==15+5*K,'forest:FrameSize','Bad member list.');
        members=repmat(struct('id',0,'role',0,'slot_index',0),1,K);
        for k=1:K,j=16+5*(k-1);members(k)=struct('id',v(b(j:j+1)),'role',v(b(j+2)),'slot_index',v(b(j+3:j+4)));end
        m=struct('train_index',v(b(1:2)),'remaining_us',v(b(3:6)),'lease_ms',v(b(7:10)),'slot_us',v(b(11:14)),'members',members);
    case 2,m=struct('capability',v(b(1:2)),'battery_mV',v(b(3:4)),'data_seq',v(b(5:6)),'data_bytes',v(b(7:8)),'sample_epoch_local',v(b(9:12)));
    case 3
        m=struct('data_seq',v(b(1:2)),'fragment_index',v(b(3)),'fragment_count',v(b(4)), ...
            'sample_epoch_local',v(b(5:8)),'sensor_flags',v(b(9:10)),'samples',b(11:end));
        assert(m.fragment_count>=1 && m.fragment_index<m.fragment_count,'forest:FrameSize','Bad fragment index.');
    case 4,m=struct('transaction',v(b(1:2)),'role',v(b(3)),'count',v(b(4)),'peer',v(b(5:6)),'sf',v(b(7)),'bw_code',v(b(8)),'calibration_id',v(b(9:10)),'start_after_us',v(b(11:14)));
    case 5
        raw=v(b(3:6));if raw>=2^31,raw=raw-2^32;end
        assert(raw>=-2^23 && raw<2^23,'forest:Raw','Invalid RAW sign extension.');
        m=struct('transaction',v(b(1:2)),'raw_result',raw,'quality_flags',v(b(7:8)),'calibration_id',v(b(9:10)),'epoch_local',v(b(11:14)));
    case 6,m=struct('acked_type',v(b(1)),'acked_seq',v(b(2:3)),'status',v(b(4)));
    case 7,m=struct('result_seq',v(b(1:2)),'action',v(b(3)),'ack_window_us',v(b(4:7)));
end
if type~=1 && type~=3,assert(numel(b)==sizes(type),'forest:FrameSize','Unexpected trailing bytes.');end
end

function b=u(x,n)
assert(isscalar(x) && isfinite(x) && x==fix(x) && x>=0 && x<2^(8*n),'forest:FrameField','Field overflow.');
b=uint8(mod(floor(double(x)./256.^(n-1:-1:0)),256));
end
function x=v(b)
x=sum(double(b).*256.^(numel(b)-1:-1:0));
end
