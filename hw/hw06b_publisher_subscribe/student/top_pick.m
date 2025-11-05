function tdp_ctraj = top_pick(top_down_pick_joints,robot,tool_frame)
%--------------------------------------------------------------------------
% Top-Pick Skill
% Produce a skill that is able to pick an imaginary object from the top.
% This skill should have four trajectory points:
% 0. Start: robot should start from some ready position
% 1. Pre-pick: approach object location, but stop some small distance (10cm) above
% 2. Pick: Perform top-down motion to approach the object (we will not close
% the fingers here)
% 3. Pre-Place: approach place location, but stop some small distance (10cm) above
% 4. Place: Perform top-down motion to place object. 
%
% Inputs:
%   - top_down_pick_joints: [double]        A 5x9 matrix consisting of 5 joint angle
%                                           configurations to achieve a top-pick skill with the panda robot.
%   - robot:                rigidBodyTree   the tree object for the robot
%   - tool_frame:           String          Name of tool frame for respective robot, i.e. "panda_hand"
%
% Outputs:
%   tdp_ctraj:              [double]        A 4x4x5 matrix composed of 5 4x4 matrices corresponding 
%                                           to the forward kinematics at each of the 5 way-points.
%----------------------------------------------------------------------
    % BEGIN_YOUR_CODE 
    error("Not implemented yet")
    % END_YOUR_CODE
end