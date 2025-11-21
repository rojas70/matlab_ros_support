
%% Main Code to Start ARM Pick and Place.
% Start the ROS connection, handle, goHome, and reset the world.
clear; close all; clc;
optns = startRobotWorld;

% Gray zone 1, easy 

% Can create a flag in optns to choose whether to do static/automated.
optns{'static'} = false;

if optns{'static'}

    % -- can set things statically (needs to identiy poses if scene changes)
    staticPickAndPlace(optns); % Not preferred. 

else
    % Automated method
    PickandPlaceARMChallenge('Zone1', optns);
    PickandPlaceARMChallenge('Zone2', optns);
end

% Yellow zone 3, medium
PickandPlaceARMChallenge('Zone3', optns);

% Red zone 4, hard
PickandPlaceARMChallenge('Zone4', optns);

% Blue zone 5, very hard
PickandPlaceARMChallenge('Zone5', optns);
