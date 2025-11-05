function matlab_pose = ros2matlabPose(p,frameAdjustmentFlag,toolAdjustmentFlag,optns)
    %----------------------------------------------------------------------
    % ros2matlabPose
    % Converts ros pose to matlab pose according to type. Handles two Pose
    % types:
    %   gazebo_msgs/GetModelStateResponse:
    %       - Position.XYZ 
    %       - Orientation.XYZW
    %   geometry_msgs/TransformStamped
    %       - Transform.Translation.XYZ
    %       - Transform.Rotation.XYZW
    % 
    % Also has two adjustments:
    % - frameAdjustment makes up for the difference between ROS/Gazebo
    % standard frames and the frames used by matlab here where x=y, y=x,
    % and z=-z. Visualize in RVIZ for world and base frame.
    %
    % - toolAdjustmentFlag:
    % flag to indicates we have fingers but have not adjusted IKs for it.
    % 
    % Inputs:
    % - p (ros structure): pose 
    % - frameAdjustment (double): adjustments: x=y, y=x, z=-z.
    % - toolAdjustmentFlag (double): tool distance 0.165. Adjusted at end
    % of program.
    % Outputs: 
    % matlab_pose (Transform): 4x4 matrix
    %
    % Extracts position into p and orientation into a quaternion iva the RVC 
    % Toolbox to instantiate UnitQuaternion and build a homogeneous
    % transform by composing orientation and translation.
    %----------------------------------------------------------------------
    
    %% Local Variables
    endEffectorAdjustment = 0;

    % Flags
    if nargin == 1
        frameAdjustmentFlag = 0;
        toolAdjustmentFlag = 0;

    else if nargin == 2
        toolAdjustmentFlag = 0;
    end

    % Extract an offset in the z-direction that translates wrist to finger tips
    if toolAdjustmentFlag
        endEffectorAdjustment = optns{'toolAdjustment'}; %0.165;
    end

    % Normal. No frame adjustment needed
    if ~frameAdjustmentFlag 

        % gazebo_msgs/GetModelStateResponse
        if strcmp(p.MessageType, 'gazebo_msgs/GetModelStateResponse')
            pos = [p.Pose.Position.X, ...
                   p.Pose.Position.Y, ...
                   (p.Pose.Position.Z - endEffectorAdjustment)];
    
            q = UnitQuaternion(p.Pose.Orientation.W, ...
                               [p.Pose.Orientation.X, ...
                                p.Pose.Orientation.Y, ...
                                p.Pose.Orientation.Z]);

        % 'geometry_msgs/TransformStamped'
        elseif strcmp(p.MessageType, 'geometry_msgs/TransformStamped')
            pos = [p.Transform.Translation.X, ...
                   p.Transform.Translation.Y, ...
                   (p.Transform.Translation.Z - endEffectorAdjustment)];
    
            q = UnitQuaternion(p.Transform.Rotation.W, ...
                               [p.Transform.Rotation.X, ...
                                p.Transform.Rotation.Y, ...
                                p.Transform.Rotation.Z]);
        end
    
    % Adjustment
    % ROS_T_MAT = rotx(-90,'deg'), or
    % ros_x=>mat_y, ros_y=>-mat_x, z=z. Or mat_x = -ros_y, mat_y=ros_x mat_z=ros_z + adjustment
    else
        % 
        if strcmp(p.MessageType, 'gazebo_msgs/GetModelStateResponse')
            pos = [-p.Pose.Position.Y, ...
                    p.Pose.Position.X, ...
                   (p.Pose.Position.Z + endEffectorAdjustment)];
    
            q = UnitQuaternion(p.Pose.Orientation.W, ...
                               [-p.Pose.Orientation.Y, ...
                                 p.Pose.Orientation.X, ...
                                (p.Pose.Orientation.Z)]);
        % 'geometry_msgs/TransformStamped'
        elseif strcmp(p.MessageType, 'geometry_msgs/TransformStamped')
            pos = [-p.Transform.Translation.Y, ...
                    p.Transform.Translation.X, ...
                    p.Transform.Translation.Z];
    
            q = UnitQuaternion(p.Transform.Rotation.W, ...
                               [-p.Transform.Rotation.Y, ...
                                p.Transform.Rotation.X, ...
                               (p.Transform.Rotation.Z)] );
        end
    end

    % Build matlab pose
    matlab_pose = transl(pos) * q.T;
end