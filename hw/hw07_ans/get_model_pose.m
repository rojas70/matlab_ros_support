function pose = get_model_pose(model_name,optns)
%--------------------------------------------------------------------------
% getModels
% This method will create an action client that talks to Gazebo's
% get_model_state action server to retrieve the pose of a given model_name wrt to the world.
%
% Inputs
% model_name (string): name of existing model in Gazebo
%
% Ouput
% models (gazebo_msgs/GetModelStateResponse): contains Pose and Twist
% structures
%--------------------------------------------------------------------------

% 01 Get robot handle
r = optns{'rHandle'};

% 02 Create model_client_msg
get_models_sate_client_msg = rosmessage(r.get_models_state_client);

% 03 Populate message with model name
get_models_sate_client_msg.ModelName = model_name;

% 04 Call client 
try
    [pose,status] = call(r.get_models_state_client,get_models_sate_client_msg);
catch
    disp('Error - model pose could not be found')
end