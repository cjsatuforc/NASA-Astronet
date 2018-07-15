classdef mav_state < robotics.ros.Message
    %mav_state MATLAB implementation of asctec_hl_comm/mav_state
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2018 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'asctec_hl_comm/mav_state' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = 'e034f695cee32efa98d5cb7e960000e2' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Constant, Access = protected)
        GeometryMsgsPoseClass = robotics.ros.msg.internal.MessageFactory.getClassForType('geometry_msgs/Pose') % Dispatch to MATLAB class for message type geometry_msgs/Pose
        GeometryMsgsVector3Class = robotics.ros.msg.internal.MessageFactory.getClassForType('geometry_msgs/Vector3') % Dispatch to MATLAB class for message type geometry_msgs/Vector3
        StdMsgsHeaderClass = robotics.ros.msg.internal.MessageFactory.getClassForType('std_msgs/Header') % Dispatch to MATLAB class for message type std_msgs/Header
    end
    
    properties (Dependent)
        Header
        Pose
        Velocity
    end
    
    properties (Access = protected)
        Cache = struct('Header', [], 'Pose', [], 'Velocity', []) % The cache for fast data access
    end
    
    properties (Constant, Hidden)
        PropertyList = {'Header', 'Pose', 'Velocity'} % List of non-constant message properties
        ROSPropertyList = {'header', 'pose', 'velocity'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = mav_state(msg)
            %mav_state Construct the message object mav_state
            import com.mathworks.toolbox.robotics.ros.message.MessageInfo;
            
            % Support default constructor
            if nargin == 0
                obj.JavaMessage = obj.createNewJavaMessage;
                return;
            end
            
            % Construct appropriate empty array
            if isempty(msg)
                obj = obj.empty(0,1);
                return;
            end
            
            % Make scalar construction fast
            if isscalar(msg)
                % Check for correct input class
                if ~MessageInfo.compareTypes(msg(1), obj.MessageType)
                    error(message('robotics:ros:message:NoTypeMatch', obj.MessageType, ...
                        char(MessageInfo.getType(msg(1))) ));
                end
                obj.JavaMessage = msg(1);
                return;
            end
            
            % Check that this is a vector of scalar messages. Since this
            % is an object array, use arrayfun to verify.
            if ~all(arrayfun(@isscalar, msg))
                error(message('robotics:ros:message:MessageArraySizeError'));
            end
            
            % Check that all messages in the array have the correct type
            if ~all(arrayfun(@(x) MessageInfo.compareTypes(x, obj.MessageType), msg))
                error(message('robotics:ros:message:NoTypeMatchArray', obj.MessageType));
            end
            
            % Construct array of objects if necessary
            objType = class(obj);
            for i = 1:length(msg)
                obj(i,1) = feval(objType, msg(i)); %#ok<AGROW>
            end
        end
        
        function header = get.Header(obj)
            %get.Header Get the value for property Header
            if isempty(obj.Cache.Header)
                obj.Cache.Header = feval(obj.StdMsgsHeaderClass, obj.JavaMessage.getHeader);
            end
            header = obj.Cache.Header;
        end
        
        function set.Header(obj, header)
            %set.Header Set the value for property Header
            validateattributes(header, {obj.StdMsgsHeaderClass}, {'nonempty', 'scalar'}, 'mav_state', 'Header');
            
            obj.JavaMessage.setHeader(header.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Header)
                obj.Cache.Header.setJavaObject(header.getJavaObject);
            end
        end
        
        function pose = get.Pose(obj)
            %get.Pose Get the value for property Pose
            if isempty(obj.Cache.Pose)
                obj.Cache.Pose = feval(obj.GeometryMsgsPoseClass, obj.JavaMessage.getPose);
            end
            pose = obj.Cache.Pose;
        end
        
        function set.Pose(obj, pose)
            %set.Pose Set the value for property Pose
            validateattributes(pose, {obj.GeometryMsgsPoseClass}, {'nonempty', 'scalar'}, 'mav_state', 'Pose');
            
            obj.JavaMessage.setPose(pose.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Pose)
                obj.Cache.Pose.setJavaObject(pose.getJavaObject);
            end
        end
        
        function velocity = get.Velocity(obj)
            %get.Velocity Get the value for property Velocity
            if isempty(obj.Cache.Velocity)
                obj.Cache.Velocity = feval(obj.GeometryMsgsVector3Class, obj.JavaMessage.getVelocity);
            end
            velocity = obj.Cache.Velocity;
        end
        
        function set.Velocity(obj, velocity)
            %set.Velocity Set the value for property Velocity
            validateattributes(velocity, {obj.GeometryMsgsVector3Class}, {'nonempty', 'scalar'}, 'mav_state', 'Velocity');
            
            obj.JavaMessage.setVelocity(velocity.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Velocity)
                obj.Cache.Velocity.setJavaObject(velocity.getJavaObject);
            end
        end
    end
    
    methods (Access = protected)
        function resetCache(obj)
            %resetCache Resets any cached properties
            obj.Cache.Header = [];
            obj.Cache.Pose = [];
            obj.Cache.Velocity = [];
        end
        
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method for shallow copy
            cpObj = copyElement@robotics.ros.Message(obj);
            
            % Clear any existing cached properties
            cpObj.resetCache;
            
            % Create a new Java message object
            cpObj.JavaMessage = obj.createNewJavaMessage;
            
            % Recursively copy compound properties
            cpObj.Header = copy(obj.Header);
            cpObj.Pose = copy(obj.Pose);
            cpObj.Velocity = copy(obj.Velocity);
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.Header = feval([obj.StdMsgsHeaderClass '.loadobj'], strObj.Header);
            obj.Pose = feval([obj.GeometryMsgsPoseClass '.loadobj'], strObj.Pose);
            obj.Velocity = feval([obj.GeometryMsgsVector3Class '.loadobj'], strObj.Velocity);
        end
    end
    
    methods (Access = ?robotics.ros.Message)
        function strObj = saveobj(obj)
            %saveobj Implements saving of message to MAT file
            
            % Return an empty element if object array is empty
            if isempty(obj)
                strObj = struct.empty;
                return
            end
            
            strObj.Header = saveobj(obj.Header);
            strObj.Pose = saveobj(obj.Pose);
            strObj.Velocity = saveobj(obj.Velocity);
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.asctec_hl_comm.mav_state.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.asctec_hl_comm.mav_state;
            obj.reload(strObj);
        end
    end
end