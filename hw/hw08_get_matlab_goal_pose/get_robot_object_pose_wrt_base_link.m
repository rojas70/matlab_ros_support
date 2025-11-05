function [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name,get_robot_gripper_pose_flag,optns)
%--------------------------------------------------------------------------
% Extract robot and model poses wrt to base_link. 
% Use gazebo service to extract poses wrt to world.
% Transform coordinate frames from world_to_base link.
% Get starting pose of robot and use its orientation to pick up object
%
% Input:
% model_name (string) - name of model available in gazebo simulation
% get_robot_gripper_pose_flag (double) - 1|0 to indicate if we want to compute robot gripper (uses tf tform)
%
% Output: 
% - mat_R_T_G [4x4] double - transformation from robot base_link to tip
% - mart_R_T_M [4x4] double -  transformation from robot base_link to obj
%--------------------------------------------------------------------------

    %% Local variables

    % Get robot handle
    r = optns{'rHandle'};

    %% Initialize variables     
    frameAdjustmentFlag = optns{'frameAdjustmentFlag'};     % Indicates matlab's base_link does not match ros. Need adjustment.
    toolAdjustmentFlag  = optns{'toolAdjustmentFlag'};      % Indicates we have fingers but have not adjusted IKs for it.    
    z_offset = optns{'z_offset'};
    % y_offset = optns{'y_offset'};
    %tf_listening_time   = r.tf_listening_time;             % Time (secs) to listen for transformation in ros
    
    % No get_robot_gripper_pose_flag provided
    if nargin == 2
        get_robot_gripper_pose_flag = 0;
    end

    
    %% 1. Get Poses from matlab wrt to World
    disp('Setting the goal...');

    % Robot's base_link and model pose wrt gazebo world origin
    W_T_R = get_model_pose('robot', optns);
    W_T_M = get_model_pose(model_name, optns);
    
    %% 2. Get Goal|Current Pose wrt to **MATLAB** base link in matlab format
    mat_W_T_R = ros2matlabPose(W_T_R, frameAdjustmentFlag, toolAdjustmentFlag, optns);
    mat_W_T_M = ros2matlabPose(W_T_M, frameAdjustmentFlag, toolAdjustmentFlag, optns); % Frame at junction with table
    
    % Change reference frame from world to robot's base_link
    mat_R_T_M = inv(mat_W_T_R)*mat_W_T_M; %#ok<MINV>

    %z_offset = 0.052; %0.052; % Can height is 5.2cm
    mat_R_T_M(3,4) = mat_R_T_M(3,4) + z_offset; % Offset along +z_base_link to simulate knowing height of top of can.
    
    
    %% 3. Modify orientation of robot pose to be a top-down pick (see tool0 vs base_link) w fingers aligned with matlab's y_world -axis
    fing_along_y = eul2tform([-pi/2 -pi 0]); % ZYX axis
    %fing_along_x = eul2tform([0 0 pi]); 

    mat_R_T_M(1:3,1:3) = fing_along_y(1:3,1:3);
            
    %% 4. Current Robot Pose in Cartesian Format
    base         = 'base_link';

    if get_robot_gripper_pose_flag
        end_effector = 'gripper_tip_link'; % When finger is properly modeled use 'gripper_tip_link'
    else
        end_effector = 'tool0';
    end

    % Get end-effector pose wrt to base via getTransform(tftree,targetframe,sourceframe), where sourceframe is the reference frame 
    try
        current_pose = getTransform(r.tftree,end_effector,base,rostime('now'),'Timeout', r.tf_listening_time);
    catch
        % Try again
        current_pose = getTransform(r.tftree,end_effector,base,rostime('now'),'Timeout', r.tf_listening_time);
    end

    % Convert gripper pose to matlab format
    mat_R_T_G = ros2matlabPose(current_pose,frameAdjustmentFlag,toolAdjustmentFlag,optns);    
    
end