function optns = startRobotWorld(masterhostIP, nodeIP, robotName)
%----------------------------------------------------------------------
% startRobotWorld
% Executes startup code before resesting the world and moving the arm.
%-----------------------------------------------------------------------
    % Init Params
    if nargin == 0
        masterhostIP = "192.168.56.101";
        robotName = "UR5e";
        nodeIP = "192.168.56.1";
    end

    % Global ROSMASTER SETUP
    rosshutdown;
    rosinit(masterhostIP,11311,"NodeHost",nodeIP);
    
    % Robot Class Handle
    if robotName == "UR5e"
         r = rosClassHandle_UR5e;
    else
        disp('Unknown handle name. Exting...');
        exit();
    end
    
   %% Global dictionary with all options
   keys   = ["debug", ...
            "toolFlag",...
            "traj_steps", ...
            "x_offset", ...
            "y_offset", ...
            "z_offset", ...
            'gripPos', ...
            "traj_duration",...
            "frameAdjustmentFlag", ...
            "toolAdjustmentFlag", ...
            "toolAdjustment", ...
            "tf_listening_time",...
            "rHandle",...
            "cleanStart"];
   
   values = {  0,...        "debug"
               0,...        "toolFlag"
               1,...        "toolFlag"
               0,...        "x_offset"
               0,...        "y_offset"
               0.2,...      "z_offset"
               0.23,...     'gripPos'
               2,...        "traj_duration"
               1,...        "frameAdjustmentFlag"
               1,...        "toolAdjustmentFlag"
               0.165,...    "toolAdjustment"
               10,...       "tf_listening_time"
               r,...        "rHandle"
               true,...     "cleanStart"
               };
    optns = dictionary(keys,values);

    %% Go Home 
    
    % Always move arm to start position and open fingers
    disp('Going home...');  % Always use display commands to inform reader of what is happening
    
    % Open fingers and move arm to qr or qz start configuration. TODO: if arm already at home, skip call. 
    goHome('qr', optns);    
    
    %% Reset the simulation
    
    disp('Resetting the world...');
    resetWorld(optns);      % reset models through a gazebo service       
    
end