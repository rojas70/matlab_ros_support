function [xyz,theta,ptCloud_vec,scene_pca_vec] = betterObjectPoses(ptCloud_tform_cam, nonPlane_tform_cam, rbgImage, bboxes, gridDownsample, base_to_cam_pose)
% the base is findObjectPoses but it is now tweaked by Gracelyn to better
% suit to use of merged point clouds

% declaring and allocating variables
numObjects = size(bboxes,1);
ptCloud_vec = cell(numObjects, 1);
scene_pca_vec = cell(numObjects, 1);
theta = [];
xyz =[];

% finding the xyz limits of the point cloud
pc_xmin = ptCloud_tform_cam.XLimits(1);
pc_xmax = ptCloud_tform_cam.XLimits(2);
pc_ymin = ptCloud_tform_cam.YLimits(1);
pc_ymax = ptCloud_tform_cam.YLimits(2);
pc_zmin = ptCloud_tform_cam.ZLimits(1);
pc_zmax = ptCloud_tform_cam.ZLimits(2);

[m,n,~] = size(rbgImage); % get the size of image

for idx= 1:numObjects
    % calculate the pixel scale for width and height
    xpixel_base_scale = (pc_xmax - pc_xmin)/n;
    ypixel_base_scale = (pc_ymax - pc_ymin)/m;
    
    % get the bouding box edges with respect to the image size/pixels
    xincrease = bboxes(idx,3)/6;
    yincrease = bboxes(idx,4)/6;
    left_column = bboxes(idx,1) - xincrease;
    right_column = bboxes(idx,1) + bboxes(idx,3) + xincrease;
    top_row = bboxes(idx,2) - yincrease;
    bottom_row = bboxes(idx,2) + bboxes(idx,4) + yincrease;
    if left_column < 1
        left_column = 1;
    end
    if right_column > n
        right_column = n;
    end
    if top_row < 1
        top_row = 1;
    end
    if bottom_row > m
        bottom_row = m;
    end

    % use bounding box locations on the image and translate them to the
    % base link reference
    bbox_roi_xmin = pc_xmin + (left_column*xpixel_base_scale);
    bbox_roi_xmax = pc_xmin + (right_column*xpixel_base_scale);
    bbox_roi_ymin = pc_ymin + (top_row*ypixel_base_scale);
    bbox_roi_ymax = pc_ymin + (bottom_row*ypixel_base_scale);

    % the roi created by the bounding box with respect to the point cloud
    bbox_roi = [bbox_roi_xmin bbox_roi_xmax bbox_roi_ymin bbox_roi_ymax pc_zmin pc_zmax];
    % find the point cloud points within the bounding box
    instanceMask = findPointsInROI(nonPlane_tform_cam,bbox_roi);
    pt_instance = select(nonPlane_tform_cam,instanceMask);
    % transform the point cloud back to base_link reference frame
    tform_to_base = rigidtform3d(base_to_cam_pose);
    pt_instance_base = pctransform(pt_instance,tform_to_base);
    pt_instance_base = pcdownsample(pt_instance_base, 'gridAverage', gridDownsample); % makes the voxel size bigger
    % remove partial point cloud of other object
    [lab_instance, num_instance] = pcsegdist(pt_instance_base, gridDownsample * sqrt(3));

    % finding which point cloud segment has the most points in it
    c = [];
    for j = 1:num_instance
        c(j) = nnz(lab_instance == j);
    end
    [~, max_labelJ] = max(c);

    % keep the point cloud segment with the most points in it
    ptCloud_vec{idx} = select(pt_instance_base, find(lab_instance==max_labelJ));
    ptScene = ptCloud_vec{idx};

    % find the zmax of the instance point cloud for the object
    xmin_scene = ptScene.XLimits(1);
    xmax_scene = ptScene.XLimits(2);
    ymin_scene = ptScene.YLimits(1);
    ymax_scene = ptScene.YLimits(2);
    zmin_scene = ptScene.ZLimits(1);
    zmax_scene = ptScene.ZLimits(2);

    % find the center
    xmid = (xmin_scene + xmax_scene)/2;
    ymid = (ymin_scene + ymax_scene)/2;
    zmid = (zmin_scene + zmax_scene)/2;
    center = [xmid ymid zmid];

    % PCA
    [coeff, score, latent] = pca(ptScene.Location);

    if (det(coeff) < 0)  % det = -1 means rigyht-handed system, so transforms into left-handed system
        coeff(:, 2) = -coeff(:, 2);
    end

    %compute scene each object PCA
    centroid= mean(ptScene.Location);
    % coeff = align2ndAxis(ptScene, coeff, centroid);%align axis by using point cloud density
    % coeff = align3rdAxis(ptScene, coeff, centroid);
    coeff = align2ndAxis(ptScene, coeff, center);%align axis by using point cloud density
    coeff = align3rdAxis(ptScene, coeff, center);
    [U,V,W] = makeUVWfromCoeff(coeff);

    % make pca struct
    scene_pca.coeff = coeff;
    scene_pca.score = score;
    scene_pca.latent = latent;
    % scene_pca.centroid = centroid;
    scene_pca.centroid = center;
    scene_pca.eulZYX = rotm2eul(coeff);
    scene_pca.UVW = [U,V,W];
    scene_pca_vec{idx} = scene_pca;
    majorAxis = [U(1), V(1), W(1)];

    %This calculates the angle between the positive x-axis ([ 1 0 0]) and the major axis of the object in an anti-clockwise direction
    theta(idx) = atan2d(dot([0 0 1],cross([ 1 0 0],majorAxis)),dot([ 1 0 0],majorAxis));
    if (theta(idx)<0)
        theta(idx) = 180 + theta(idx);
    end
    
    roi = [centroid(1)-0.01 centroid(1)+0.01 centroid(2)-0.01 centroid(2)+0.01 -inf inf];
    indices = findPointsInROI(ptScene,roi);
    pt_surface_patch = select(ptScene,indices);
    z = mean(pt_surface_patch.Location(:,3));
    %xyz(idx,:) = [centroid(1) centroid(2) z];
    xyz(idx,:) = [center(1) center(2) center(3)];
end
end

%% Helper functions

function coeff_aligned = align2ndAxis(ptCloud, coeff, center)
%This function is for internal use only and may be removed in the future.
ptCloud_aligned = pctransform(ptCloud, rigid3d(eye(3), -center));
ptCloud_aligned = pctransform(ptCloud_aligned, rigid3d(coeff, [0 0 0]));

points_y = ptCloud_aligned.Location(:, 2);
posy = points_y > 0;

moments_pos = sum(points_y(posy) .^ 2);
moments_neg = sum(points_y(~posy) .^ 2);

coeff_rot = coeff;
coeff_rot(:, 2) = -coeff(:, 2);
coeff_rot(:, 3) = -coeff(:, 3);

if (moments_pos > moments_neg)
    coeff_aligned = coeff;
else
    coeff_aligned = coeff_rot;
end
end


function coeff_aligned = align3rdAxis(ptCloud, coeff, center)
%This function is for internal use only and may be removed in the future.
ptCloud_aligned = pctransform(ptCloud, rigid3d(eye(3), -center));
ptCloud_aligned = pctransform(ptCloud_aligned, rigid3d(coeff, [0 0 0]));

points_z = ptCloud_aligned.Location(:, 3);
posy = points_z > 0;

moments_pos = sum(points_z(posy) .^ 2);
moments_neg = sum(points_z(~posy) .^ 2);

coeff_rot = coeff;
coeff_rot(:, 1) = -coeff(:, 1);
coeff_rot(:, 3) = -coeff(:, 3);

if (moments_pos > moments_neg)
    coeff_aligned = coeff;
else
    coeff_aligned = coeff_rot;
end
end


function [U,V,W] = makeUVWfromCoeff(coeff)
visual_fitting = 0.05;
% 1st principal axis
U1 = coeff(1, 1) * visual_fitting;
V1 = coeff(2, 1) * visual_fitting;
W1 = coeff(3, 1) * visual_fitting;

% 2nd principal axis
U2 = coeff(1, 2) * visual_fitting;
V2 = coeff(2, 2) * visual_fitting;
W2 = coeff(3, 2) * visual_fitting;

% 3rd principal axis
U3 = coeff(1, 3) * visual_fitting;
V3 = coeff(2, 3) * visual_fitting;
W3 = coeff(3, 3) * visual_fitting;
U = [U1;U2;U3];
V = [V1;V2;V3];
W = [W1;W2;W3];
end





