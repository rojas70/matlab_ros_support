function mat_R_T_G = get_gripper_pose(optns,find_alignment)
% Return the gripper pose. 
% Since we are computing the pose wrt to tool0, we do not have information
% of the fingers. The 'fing_alignment' will tell whether the fingers are
% aligned about the x-axis of the y-axis wrt matlab_coordinates for the
% base_link. By default x-axis.

    %% Local variables
    if nargin == 1
        find_alignment = 'x';
    end

    % Indicates matlab's base_link does not match ros. Need adjustment.
    frameAdjustmentFlag = optns{'frameAdjustmentFlag'};    
    
    % Indicates we have fingers but have not adjusted IKs for it, so a manual vertical offset is needed.    
    toolAdjustmentFlag  = optns{'toolAdjustmentFlag'}; 
    
    % List to tftree
    r = optns{'rHandle'};
    tf_listening_time   = r.tf_listening_time;    
    
    % Get gripper transform
    tftree       = rostf('DataFormat','struct');     
    base         = 'base_link';
    end_effector = 'tool0';                         % When finger is properly modeled use 'gripper_tip_link'
        
    % Get end-effector pose wrt to base via getTransform(tftree,targetframe,sourceframe), where sourceframe is the reference frame 
    % However, that order of parameters does not give the right answer. Need to reverse them!!!
    try waitForTransform(tftree,'base_link','tool0',5);
        current_pose = getTransform(tftree,base,end_effector,rostime('now'),'Timeout', r.tf_listening_time);
    catch
        % Try again
        current_pose = getTransform(tftree,base,end_effector,rostime('now'),'Timeout', r.tf_listening_time);
    end  
        
    % Convert gripper pose to matlab format
    mat_R_T_G = ros2matlabPose(current_pose,frameAdjustmentFlag,toolAdjustmentFlag,optns); 

    %% Adjust orientation of gripper
    if strcmpi(find_alignment,'x')
        ori = eul2tform([0,0,pi]);  
    % 'y'
    else
        ori = eul2tform([-pi/2, pi, 0]);
    end

    mat_R_T_G(1:3,1:3) = ori(1:3,1:3);
end