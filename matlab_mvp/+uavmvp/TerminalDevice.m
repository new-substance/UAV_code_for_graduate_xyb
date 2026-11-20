classdef TerminalDevice < handle
    %TERMINALDEVICE Logical radio state. No receive-power claim.
    % Stopped devices still accept duplicate STOP for idempotent ACK; they do
    % not resume ranging or respond to same-mission group controls.
    properties (Access=private)
        address
        mission_id
        awake=false
        stopped=false
    end
    methods
        function obj=TerminalDevice(address,mission)
            obj.address=address;obj.mission_id=mission;
        end
        function packet=receive_group(obj,control)
            packet=[];
            if isempty(control) || obj.stopped || control.mission_id~=obj.mission_id,return;end
            slot=find([control.schedule.address]==obj.address,1);
            if isempty(slot),return;end
            if strcmp(control.kind,'GROUP_WAKE'),obj.awake=true;
            elseif ~strcmp(control.kind,'GROUP_POLL'),return;end
            if ~obj.awake,return;end
            entry=control.schedule(slot);
            packet=struct('mission_id',obj.mission_id,'address',obj.address,'kind','ID_REPLY', ...
                'session_id',entry.session_id,'sequence',entry.sequence,'crc_valid',true, ...
                'reply_offset_s',entry.reply_offset_s,'device_state','AWAKE');
        end
        function allowed=can_respond_range(obj,request)
            allowed=obj.awake && ~obj.stopped && request.mission_id==obj.mission_id && request.address==obj.address;
        end
        function packet=receive_stop(obj,command)
            packet=[];
            if command.mission_id~=obj.mission_id || command.address~=obj.address || ...
                    ~strcmp(command.kind,'STOP') || ~command.crc_valid,return;end
            obj.stopped=true;obj.awake=false;packet=command;packet.kind='STOP_ACK';
        end
    end
end
