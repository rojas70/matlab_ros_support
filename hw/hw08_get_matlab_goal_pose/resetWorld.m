function resetWorld(optns)
%-------------------------------------------------------------------------- 
% resetWorld()
% Calls Gazebo service to reset the world
% Input: (dict) optns
% Output: None
%-------------------------------------------------------------------------- 
    disp('Resetting the world...');
    
    % Get robot handle
    r = optns{'rHandle'};
    
    % Create Empty Simulation message
    ros_client_msg = rosmessage(r.res_client);
    
    % Call reset service
    status = waitForServer(r.res_client);    
    if status
        try
            [testresp,status,statustext] = call(r.res_client, ros_client_msg,"Timeout",3);
        catch
            disp('Error - models not listed...')    
        
        end  
    end
end