classdef TerminalDevice < uavmvp.TerminalDevice
    %TERMINALDEVICE One final ACK emission, then whole-device deep sleep.
    properties (Access=private)
        sleeping=false
    end
    methods
        function obj=TerminalDevice(address,mission)
            obj@uavmvp.TerminalDevice(address,mission);
        end
        function packet=receive_group(obj,control)
            packet=[];if obj.sleeping,return;end
            packet=receive_group@uavmvp.TerminalDevice(obj,control);
        end
        function ok=can_respond_range(obj,request)
            ok=~obj.sleeping && can_respond_range@uavmvp.TerminalDevice(obj,request);
        end
        function packet=receive_stop(obj,command)
            packet=[];if obj.sleeping,return;end
            packet=receive_stop@uavmvp.TerminalDevice(obj,command);
            if ~isempty(packet),obj.sleeping=true;end
        end
        function state=offline_state(obj)
            state=struct('deep_sleep',obj.sleeping,'environment_sampling',false, ...
                'receiver_enabled',~obj.sleeping,'later_monitoring_wake','outside_this_stage');
        end
    end
end
