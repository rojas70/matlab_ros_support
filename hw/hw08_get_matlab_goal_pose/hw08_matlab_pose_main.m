%% HW08 - Get Matlab Goal Pose

%% 00 Connect to ROS (use your own masterhost IP address)
clc
clear
rosshutdown;
pause(2);       

%% Set IP address for master and node:
masterHostIP = % **Complete Code** 
nodeHostIP = % **Complete Code** 
rosinit(masterHostIP, 11311, "NodeHost",nodeHostIP);

%% ROS Class handle

% r will contains all publisher/subscriber/service/action/kinematic/tf info
disp("Creating Robot Handle...");
r = rosClassHandle_UR5e;

%% Options 

% Create global dictionary. Passed to all functions. Includes robot handle
keys   = ["debug", "toolFlag", "z_offset", "frameAdjustmentFlag", "toolAdjustmentFlag", "toolAdjustment", "rHandle"];
values = {      0,          0, 0.09,       1,                      1,                   0.165,         r};

% Instantiate the dictionary: values can be access via {}, i.e. optns{'key'}
disp("Creating dictionary...");
optns = dictionary(keys,values);

%% 02 Reset the simulation

disp('Resetting the world...');
resetWorld(optns);      % reset models through a gazebo service

%% 04 Get Model Poses wrt to Gazebo

type = 'gazebo'; % gazebo, ptcloud, cam, manual
disp('Getting Gazebo object goal pose(s)...')

% Get models from Gazebo
models = getModels(optns);

% Number of models to pick (you can hard code or randomize)
n = 7; % Aiming for a fixed and simple object in the middle of the workspace.

% Green Bin Location - Extracted empirically
greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0];

%% 03 Get Poses in Matlab format: (i) robot_base-to-model   (mat_R_T_M)
%%                                (ii)robot_base-to-gripper (mat_R_T_G)

% Get Model Names
model_name = models.ModelNames{23+n};

% Get Model pose
fprintf('Picking up model: %s \n',model_name);
get_robot_gripper_pose_flag = 0; % 0 - no model of fingers available
[mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name,get_robot_gripper_pose_flag,optns);

display("Base to tip Transform: ");
disp(mat_R_T_G);

display("Base to Object Transform: ");
disp(mat_R_T_M);