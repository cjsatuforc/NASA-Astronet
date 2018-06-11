classdef CustomMsgConsts
    %CustomMsgConsts This class stores all message types
    %   The message types are constant properties, which in turn resolve
    %   to the strings of the actual types.
    
    %   Copyright 2014-2018 The MathWorks, Inc.
    
    properties (Constant)
        asctec_hl_comm_DoubleArrayStamped = 'asctec_hl_comm/DoubleArrayStamped'
        asctec_hl_comm_GpsCustom = 'asctec_hl_comm/GpsCustom'
        asctec_hl_comm_GpsCustomCartesian = 'asctec_hl_comm/GpsCustomCartesian'
        asctec_hl_comm_MavCtrlSrv = 'asctec_hl_comm/MavCtrlSrv'
        asctec_hl_comm_MavCtrlSrvRequest = 'asctec_hl_comm/MavCtrlSrvRequest'
        asctec_hl_comm_MavCtrlSrvResponse = 'asctec_hl_comm/MavCtrlSrvResponse'
        asctec_hl_comm_MotorSpeed = 'asctec_hl_comm/MotorSpeed'
        asctec_hl_comm_PositionWithCovarianceStamped = 'asctec_hl_comm/PositionWithCovarianceStamped'
        asctec_hl_comm_WaypointAction = 'asctec_hl_comm/WaypointAction'
        asctec_hl_comm_WaypointActionFeedback = 'asctec_hl_comm/WaypointActionFeedback'
        asctec_hl_comm_WaypointActionGoal = 'asctec_hl_comm/WaypointActionGoal'
        asctec_hl_comm_WaypointActionResult = 'asctec_hl_comm/WaypointActionResult'
        asctec_hl_comm_WaypointFeedback = 'asctec_hl_comm/WaypointFeedback'
        asctec_hl_comm_WaypointGoal = 'asctec_hl_comm/WaypointGoal'
        asctec_hl_comm_WaypointResult = 'asctec_hl_comm/WaypointResult'
        asctec_hl_comm_Wgs84ToEnu = 'asctec_hl_comm/Wgs84ToEnu'
        asctec_hl_comm_Wgs84ToEnuRequest = 'asctec_hl_comm/Wgs84ToEnuRequest'
        asctec_hl_comm_Wgs84ToEnuResponse = 'asctec_hl_comm/Wgs84ToEnuResponse'
        asctec_hl_comm_mav_ctrl = 'asctec_hl_comm/mav_ctrl'
        asctec_hl_comm_mav_ctrl_motors = 'asctec_hl_comm/mav_ctrl_motors'
        asctec_hl_comm_mav_ctrl_motorsRequest = 'asctec_hl_comm/mav_ctrl_motorsRequest'
        asctec_hl_comm_mav_ctrl_motorsResponse = 'asctec_hl_comm/mav_ctrl_motorsResponse'
        asctec_hl_comm_mav_ekf = 'asctec_hl_comm/mav_ekf'
        asctec_hl_comm_mav_imu = 'asctec_hl_comm/mav_imu'
        asctec_hl_comm_mav_rcdata = 'asctec_hl_comm/mav_rcdata'
        asctec_hl_comm_mav_state = 'asctec_hl_comm/mav_state'
        asctec_hl_comm_mav_status = 'asctec_hl_comm/mav_status'
    end
    
    methods (Static, Hidden)
        function messageList = getMessageList
            %getMessageList Generate a cell array with all message types.
            %   The list will be sorted alphabetically.
            
            persistent msgList
            if isempty(msgList)
                msgList = cell(24, 1);
                msgList{1} = 'asctec_hl_comm/DoubleArrayStamped';
                msgList{2} = 'asctec_hl_comm/GpsCustom';
                msgList{3} = 'asctec_hl_comm/GpsCustomCartesian';
                msgList{4} = 'asctec_hl_comm/MavCtrlSrvRequest';
                msgList{5} = 'asctec_hl_comm/MavCtrlSrvResponse';
                msgList{6} = 'asctec_hl_comm/MotorSpeed';
                msgList{7} = 'asctec_hl_comm/PositionWithCovarianceStamped';
                msgList{8} = 'asctec_hl_comm/WaypointAction';
                msgList{9} = 'asctec_hl_comm/WaypointActionFeedback';
                msgList{10} = 'asctec_hl_comm/WaypointActionGoal';
                msgList{11} = 'asctec_hl_comm/WaypointActionResult';
                msgList{12} = 'asctec_hl_comm/WaypointFeedback';
                msgList{13} = 'asctec_hl_comm/WaypointGoal';
                msgList{14} = 'asctec_hl_comm/WaypointResult';
                msgList{15} = 'asctec_hl_comm/Wgs84ToEnuRequest';
                msgList{16} = 'asctec_hl_comm/Wgs84ToEnuResponse';
                msgList{17} = 'asctec_hl_comm/mav_ctrl';
                msgList{18} = 'asctec_hl_comm/mav_ctrl_motorsRequest';
                msgList{19} = 'asctec_hl_comm/mav_ctrl_motorsResponse';
                msgList{20} = 'asctec_hl_comm/mav_ekf';
                msgList{21} = 'asctec_hl_comm/mav_imu';
                msgList{22} = 'asctec_hl_comm/mav_rcdata';
                msgList{23} = 'asctec_hl_comm/mav_state';
                msgList{24} = 'asctec_hl_comm/mav_status';
            end
            
            messageList = msgList;
        end
        
        function serviceList = getServiceList
            %getServiceList Generate a cell array with all service types.
            %   The list will be sorted alphabetically.
            
            persistent svcList
            if isempty(svcList)
                svcList = cell(3, 1);
                svcList{1} = 'asctec_hl_comm/MavCtrlSrv';
                svcList{2} = 'asctec_hl_comm/Wgs84ToEnu';
                svcList{3} = 'asctec_hl_comm/mav_ctrl_motors';
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            serviceList = svcList;
        end
        
        function actionList = getActionList
            %getActionList Generate a cell array with all action types.
            %   The list will be sorted alphabetically.
            
            persistent actList
            if isempty(actList)
                actList = cell(1, 1);
                actList{1} = 'asctec_hl_comm/Waypoint';
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            actionList = actList;
        end
    end
end
