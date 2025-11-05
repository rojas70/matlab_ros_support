function pose = get_model_pose(model_name,optns)
%--------------------------------------------------------------------------
% getModels
% This method will create a service client that talks to Gazebo's
% get_model_state action server to retrieve the pose of a given model_name wrt to the world.
%
% Inputs
%   - model_name (string): name of existing model in Gazebo
%   - optns      (dictionary ): dictionary with different key-value pairs
%
% Ouput
%   - pose (gazebo_msgs/GetModelStateResponse): contains Pose and Twist
% structures
%--------------------------------------------------------------------------

% 01 Get robot handle
% **Complete Code**

% 02 Create model_client_msg
% **Complete Code**

% 03 Populate message with model name
% **Complete Code**

% 04 Call client 
try
    [pose,status] = call(r.get_models_state_client,get_models_sate_client_msg);
catch
    disp('Error - model pose could not be found')
end