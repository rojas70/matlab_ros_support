% *********************************************************************
% This function returns a specific pose configuration (i.e., joint angles)
% for the robot arm depending on the specified inspection zone. 
% The returned configurations are used when the robot needs to move to a 
% certain start location in Gazebo for object pickup in the ARM Competition.
%
% USAGE:
%   StartZoneLocation = inspectZone(zoneInspect)
%       - zoneInspect : A string specifying which zone's start location is desired
%       - StartZoneLocation : A 1x6 matrix of joint angles representing the 
%                            robot's configuration for that zone
% 
% EXAMPLE:
%   zone = "Zone3";
%   startConfiguration = inspectZone(zone);
%   % The robot can then be commanded to move to this startConfiguration
%
% Note:
% As these values are manual they are subject to change based on given
% scenes that change from year-to-year in the competition. These decisions
% are based on point-cloud coverage and image annotation view-angles.
% *********************************************************************

function [start_q] = returnZoneJointConfig(zoneInspect)

    %% 'Zone 1'
    if strcmp(zoneInspect, "Zone1")       
        start_q = [0.7,   -0.2818,    1.9993,   -1.7175,   -0.0003,    0.6482];
    
    %% 'Zone 2'
    elseif strcmp(zoneInspect, "Zone2")
        start_q = [-1.8380,   -0.0978,    1.8084,   -1.7105,    6.2830,     4.4452];


    %% 'Zone 3'
    elseif strcmp(zoneInspect, "Zone3")
        start_q = [-0.4400,   -0.7014,    2.0136,   -1.3122,    0.0002,   -0.4400];
    
    %% 'Zone 4'
    elseif strcmp(zoneInspect, "Zone4")
        start_q = [-0.1453,     0.4926, 0.8193,     -1.3119,    0.0,        -0.1453];
    
    %% 'Zone 4-Pouch'
    elseif strcmp(zoneInspect, "Zone4Pouch")
        start_q = [-0.1865,    0.1104,    1.7119,   -1.8223,   -0.0000,     -0.1865];
        
    %% 'Zone 5'
    elseif strcmp(zoneInspect, "Zone5")
         start_q =  [-0.9376,    0.1748,    1.4563,   -1.6311,    0.0001,    0.6332];
    end   
end
