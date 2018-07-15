classdef CustomMsgConsts
    %CustomMsgConsts This class stores all message types
    %   The message types are constant properties, which in turn resolve
    %   to the strings of the actual types.
    
    %   Copyright 2016 The MathWorks, Inc.
    
    properties (Constant)
        vicon_bridge_Marker = 'vicon_bridge/Marker'
        vicon_bridge_Markers = 'vicon_bridge/Markers'
        vicon_bridge_TfDistortInfo = 'vicon_bridge/TfDistortInfo'
        vicon_bridge_viconCalibrateSegment = 'vicon_bridge/viconCalibrateSegment'
        vicon_bridge_viconCalibrateSegmentRequest = 'vicon_bridge/viconCalibrateSegmentRequest'
        vicon_bridge_viconCalibrateSegmentResponse = 'vicon_bridge/viconCalibrateSegmentResponse'
        vicon_bridge_viconGrabPose = 'vicon_bridge/viconGrabPose'
        vicon_bridge_viconGrabPoseRequest = 'vicon_bridge/viconGrabPoseRequest'
        vicon_bridge_viconGrabPoseResponse = 'vicon_bridge/viconGrabPoseResponse'
    end
    
    methods (Static, Hidden)
        function messageList = getMessageList
            %getMessageList Generate a cell array with all message types.
            %   The list will be sorted alphabetically.
            
            persistent msgList
            if isempty(msgList)
                msgList = cell(9, 1);
                msgList{1} = 'vicon_bridge/Marker';
                msgList{2} = 'vicon_bridge/Markers';
                msgList{3} = 'vicon_bridge/TfDistortInfo';
                msgList{4} = 'vicon_bridge/viconCalibrateSegment';
                msgList{5} = 'vicon_bridge/viconCalibrateSegmentRequest';
                msgList{6} = 'vicon_bridge/viconCalibrateSegmentResponse';
                msgList{7} = 'vicon_bridge/viconGrabPose';
                msgList{8} = 'vicon_bridge/viconGrabPoseRequest';
                msgList{9} = 'vicon_bridge/viconGrabPoseResponse';
            end
            
            messageList = msgList;
        end
        
        function serviceList = getServiceList
            %getServiceList Generate a cell array with all service types.
            %   The list will be sorted alphabetically.
            
            persistent svcList
            if isempty(svcList)
                svcList = cell(2, 1);
                svcList{1} = 'vicon_bridge/viconCalibrateSegment';
                svcList{2} = 'vicon_bridge/viconGrabPose';
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            serviceList = svcList;
        end
    end
end
