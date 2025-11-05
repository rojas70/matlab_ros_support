function models = getModels(optns)
%-------------------------------------------------------------------------- 
% getModels
% This method will create an action client that talks to Gazebo's
% get_world_properties to get all models.
%
% Inputs: (dict) optns
% Output: (gazebo_msgs/GetWorldPropertiesResponse cell): models
%--------------------------------------------------------------------------
    % Get robot handle
    r = optns{'rHandle'};
        
    % 01 Create model_client_msg 
    get_models_client_msg = rosmessage(r.get_models_client);
    
    % 03 Call client 
    status = waitForServer(r.get_models_client);
    if status
        try
            [models,~,~] = call(r.get_models_client,get_models_client_msg);            
        catch
            disp('Error - models not listed...')    
        
        end
    end

end