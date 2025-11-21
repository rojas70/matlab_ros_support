function [zoneComplete] = PickandPlaceARMChallenge(zoneInspect, optns)   
%--------------------------------------------------------------------------
% PickandPlaceArmChallenge
% Divides the workspace into different zones according to Robocup
% regulations.
%
% Strategy seeks to position the manipulator at an advantageous
% configuration from which to take images for object identification and
% subsequent pose estimation.
%
% After identifying one object or objects, pick and place them.
%
% Note:
% This strategy can be improved in multiple ways. Are start zones optimal?
% Should you count the total number of objects and decrease count with each
% pick to ensure you have cleared an area? Should you visualize a scene
% after each pick attempt?
%
% TODO: pick function can be optimized for new objects like spam/markers
% (only to pursue if you train the Yolo Network to identify these.
%--------------------------------------------------------------------------
    
    % Inspect the zone and get the number of subZones and joint configurations for each subZone
    [startZone] = returnZoneJointConfig(zoneInspect);
    
    % Move the robot to the inspected object's joint configuration
    fprintf('01 Moving to zone %s at config %s...\n', zoneInspect, strjoin(string(startZone)) ); 
    moveToQ("Custom",optns,startZone);
    
    % Capture an image of the zone and detect objects
    disp('02 Object Detection...\n');
    [bboxes, ~, labeled, numOfObjects, myImg, annotatedImage] = getLabeledImg(zoneInspect,optns);
    
    % Capture a point cloud of the zone
    disp('03 Computing Merged Point Cloud...\n');
    [ptCloud_pic, nonPlane_pic, ptCloud_table, base_to_cam_pose, cam_to_base_pose] = getMergedPTC(zoneInspect,optns); 
    
    % Get the pose of each detected object
    disp('04 Obtaining object poses...\n');
    objectData = getObjectData(ptCloud_pic, nonPlane_pic, myImg, bboxes, numOfObjects, base_to_cam_pose, cam_to_base_pose, labeled);     

    % Iterate over each detected object
    numObjects = size(objectData,1);
    fprintf('I have identified %d objects...\n', numObjects);
    for j = 1:numObjects

        % pick
        disp('05 Beginning top-down pick from center....\n');
        moveToQ("Custom",optns,startZone);
        pick("topdown",objectData(j,:),optns);
        
        % place
        disp('06 Beginning place....\n');
        moveToQ("Custom",optns,startZone);
        place("topdown",objectData(j,1),optns); % label for knowing which bin to go to
    
    end
    pause(5);

end
