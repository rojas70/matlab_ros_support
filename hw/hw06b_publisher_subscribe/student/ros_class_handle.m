classdef ros_class_handle
    %ROS_CLASS_HANDLE  Example MATLAB class integrating ROS and Robotics System Toolbox.
    %
    %   A MATLAB class defines a blueprint for creating objects that combine
    %   data (stored in *properties*) and functionality (implemented through
    %   *methods*). Each object created from a class has its own copy of the
    %   properties, and can call the methods defined by the class.
    %
    %   The ROS_CLASS_HANDLE class demonstrates how to encapsulate
    %   Robot Operating System (ROS) communication and robot modeling in
    %   a single MATLAB object. When instantiated, it:
    %
    %     • Creates ROS publishers and subscribers for joint state messages.
    %     • Loads a Franka Emika Panda robot model using the Robotics System Toolbox.
    %     • Stores the robot's home configuration as the initial joint configuration.
    %     • Sets up an inverse kinematics (IK) solver and default weight parameters.
    %
    %   PROPERTIES:
    %       joint_state_pub     - ROS publisher for "/matlab_joint_states" topic.
    %       joint_state_sub     - ROS subscriber for "/joint_states" topic.
    %       robot               - Rigid body tree model of the Franka Emika Panda robot.
    %       initialRobotJConfig - Home joint configuration of the robot.
    %       ik                  - Inverse kinematics solver object.
    %       ik_weights          - Weighting factors for IK solution optimization.
    %
    %   EXAMPLE:
    %       r = ros_class_handle;
    %       jointMsg = rosmessage(r.joint_state_pub);
    %       send(r.joint_state_pub, jointMsg);
    %
    %   See also: rospublisher, rossubscriber, loadrobot, inverseKinematics
    
    properties

        % Publishers-Subscribers
        joint_state_pub;
        joint_state_sub;
        
        % Robot
        robot;
        initialRobotJConfig;

        % IK
        ik;
        ik_weights;
    end

    methods 
        function r = ros_class_handle

            % Set r as a structure. Create fields with values as:
            % r.field1 = val1
            % r.field2 = val2
            r.joint_state_pub         = % Fill your rospublisher...
            r.joint_state_sub         = % Fill your rossubscriber(...

            % Robot
            r.robot = % Fill your loadrobot(...
            r.initialRobotJConfig     = % Set to r.robot's home configuration

            % Numerical IKs
            r.ik                      = % Set to numerical inverse kinematics for your r.robot 
            r.ik_weights              = [0.25, 0.25, 0.25, 0.1, 0.1, 0.1];  
        end
    end
end