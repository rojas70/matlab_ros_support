function [xyz,theta,ptCloud_vec,scene_pca_vec] = betterObjectPoses(ptCloud_tform_cam, nonPlane_tform_cam, rbgImage, bboxes, gridDownsample, base_to_cam_pose)
% betterObjectPoses  Estimate per-object 3D poses from a merged, de-planed point cloud.
%
% This function takes a scene point cloud and 2D detection boxes and returns,
% for each detected object, a cleaned instance point cloud, its principal
% axes (via PCA), a yaw-like in-plane orientation angle, and a representative
% 3D position. It assumes you already removed the dominant plane (e.g., table)
% and that point clouds are expressed in the camera frame, with a known
% base_link <- camera transform.
%
% Inputs
%   ptCloud_tform_cam   : pointCloud object. Scene point cloud in camera frame
%                         AFTER any global transforms/merging. Used for ROI limits.
%   nonPlane_tform_cam  : pointCloud object. Same scene but with plane removed
%                         (non-plane points only), camera frame.
%   rbgImage            : MxNx3 uint8 image associated with the point cloud
%                         (typo kept as given: "rbgImage"). Used for bbox sizing.
%   bboxes              : Kx4 matrix of [x y w h] bounding boxes in image pixels
%                         (e.g., from a 2D detector like YOLO). K = number of objects.
%   gridDownsample      : scalar (meters). Voxel size for gridAverage downsampling
%                         and for segmentation distance threshold scaling.
%   base_to_cam_pose    : 4x4 homogeneous transform (double). Converts camera-frame
%                         points into base_link via rigidtform3d(base_to_cam_pose).
%
% Outputs
%   xyz           : Kx3 array. Representative object positions [x y z] in base_link.
%                   Here we use the PCA-aligned center of each instance selection.
%   theta         : 1xK vector. In-plane orientation angle (degrees) around +z,
%                   measured anti-clockwise from +x to object's major axis,
%                   mapped to [0, 180] for symmetry.
%   ptCloud_vec   : Kx1 cell. Each cell is a pointCloud of the dominant instance
%                   segment (largest cluster) for that bbox, in base_link.
%   scene_pca_vec : Kx1 cell. Each cell contains a struct with PCA results:
%                   .coeff (3x3), .score, .latent, .centroid, .eulZYX, .UVW
%
% Assumptions / Notes
%   1) Camera intrinsics are not directly used; 2D bboxes are expanded and
%      mapped into 3D XY via a simple proportional mapping using scene XY limits.
%   2) The largest cluster inside each ROI is considered the target instance.
%   3) PCA orientation is post-processed by align2ndAxis / align3rdAxis to
%      reduce axis flips using mass moments above/below axes.
%   4) theta is computed from the PCA "major axis" projected to XY.
%   5) If a bbox touches scene borders, it is clamped to the image size.
%
% Example
%   [xyz,theta,pcs,pcaS] = betterObjectPoses(pcScene, pcNoPlane, img, bboxes, 0.01, T_base_cam);
%   scatter3(xyz(:,1), xyz(:,2), xyz(:,3), 'filled'); grid on; axis equal
%---------------------------------------------------------------------------------
    
    % -------------------------------------------------------------------------
    % Preallocate outputs
    % -------------------------------------------------------------------------
    numObjects = size(bboxes,1);
    ptCloud_vec = cell(numObjects, 1);
    scene_pca_vec = cell(numObjects, 1);
    theta = [];
    xyz =[];
    
    % -------------------------------------------------------------------------
    % Scene limits (camera frame): used to convert image bboxes into 3D XY ROI
    % -------------------------------------------------------------------------
    pc_xmin = ptCloud_tform_cam.XLimits(1);
    pc_xmax = ptCloud_tform_cam.XLimits(2);
    pc_ymin = ptCloud_tform_cam.YLimits(1);
    pc_ymax = ptCloud_tform_cam.YLimits(2);
    pc_zmin = ptCloud_tform_cam.ZLimits(1);
    pc_zmax = ptCloud_tform_cam.ZLimits(2);
    
    [m,n,~] = size(rbgImage); % get the size of image
    
    for idx= 1:numObjects
        % ---------------------------------------------------------------------
        % Calculate the pixel scale for width and height and
        % Map 2D bbox (expanded slightly) to a 3D ROI in camera XY (full Z span)    
        % ---------------------------------------------------------------------
        xpixel_base_scale = (pc_xmax - pc_xmin)/n;
        ypixel_base_scale = (pc_ymax - pc_ymin)/m;
        
        % Gentle expansion of the ROI to capture more object points near edges    
        xincrease = bboxes(idx,3)/6;
        yincrease = bboxes(idx,4)/6;
        
        left_column = bboxes(idx,1) - xincrease;
        right_column = bboxes(idx,1) + bboxes(idx,3) + xincrease;
        
        top_row = bboxes(idx,2) - yincrease;
        bottom_row = bboxes(idx,2) + bboxes(idx,4) + yincrease;
        
        % Clamp to image bounds    
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
    
        % ---------------------------------------------------------------------
        % Convert image ROI -> camera-frame XY ROI (Z uses full scene span)    bbox_roi_xmin = pc_xmin + (left_column*xpixel_base_scale);
        % ---------------------------------------------------------------------    
        bbox_roi_xmax = pc_xmin + (right_column*xpixel_base_scale);
        
        bbox_roi_ymin = pc_ymin + (top_row*ypixel_base_scale);
        bbox_roi_ymax = pc_ymin + (bottom_row*ypixel_base_scale);
    
        bbox_roi = [bbox_roi_xmin bbox_roi_xmax bbox_roi_ymin bbox_roi_ymax pc_zmin pc_zmax];
        
        % ---------------------------------------------------------------------
        % Select non-plane points inside ROI (camera frame), then go to base_link
        % ---------------------------------------------------------------------
        instanceMask = findPointsInROI(nonPlane_tform_cam,bbox_roi);
        pt_instance = select(nonPlane_tform_cam,instanceMask);
        
        % Camera -> base_link
        tform_to_base = rigidtform3d(base_to_cam_pose);
        pt_instance_base = pctransform(pt_instance,tform_to_base);
        
        % Downsample to stabilize PCA and speed up clustering    
        pt_instance_base = pcdownsample(pt_instance_base, 'gridAverage', gridDownsample); % makes the voxel size bigger
    
        % ---------------------------------------------------------------------
        % Split points into clusters/groups (separates multiple objects if present); 
        % Keep the largest one as the "instance"
        % Distance threshold ~ voxel diagonal for conservative grouping
        % ---------------------------------------------------------------------
        [lab_instance, num_instance] = pcsegdist(pt_instance_base, gridDownsample * sqrt(3));
    
        % Count points per label; choose the label with the most points    c = [];
        for j = 1:num_instance
            c(j) = nnz(lab_instance == j);
        end
        [~, max_labelJ] = max(c);
    
        % Instance cloud for this bbox (base_link frame)
        ptCloud_vec{idx} = select(pt_instance_base, find(lab_instance==max_labelJ));
        ptScene = ptCloud_vec{idx};
    
        % ---------------------------------------------------------------------
        % Compute bounding box center (in base_link) as a stable representative
        % ---------------------------------------------------------------------
        xmin_scene = ptScene.XLimits(1);
        xmax_scene = ptScene.XLimits(2);
        
        ymin_scene = ptScene.YLimits(1);
        ymax_scene = ptScene.YLimits(2);
        
        zmin_scene = ptScene.ZLimits(1);
        zmax_scene = ptScene.ZLimits(2);
    
        % Find the center
        xmid = (xmin_scene + xmax_scene)/2;
        ymid = (ymin_scene + ymax_scene)/2;
        zmid = (zmin_scene + zmax_scene)/2;
        center = [xmid ymid zmid];
    
        % ---------------------------------------------------------------------
        % PCA for orientation; fix handedness and align axes using mass moments
        % ---------------------------------------------------------------------
        [coeff, score, latent] = pca(ptScene.Location);
    
        % Ensure a right-handed orientation basis (det should be +1 for a rotation)
        if (det(coeff) < 0)  
            coeff(:, 2) = -coeff(:, 2);
        end
    
        % Optional: density-based axis alignment to reduce random sign flips
        % Using the object center rather than centroid to stabilize on partial views
        %----------------------------------------------------------------------
        %centroid = mean(ptScene.Location);
        %coeff = align2ndAxis(ptScene, coeff, centroid);%align axis by using point cloud density
        %coeff = align3rdAxis(ptScene, coeff, centroid);
        %----------------------------------------------------------------------
        coeff = align2ndAxis(ptScene, coeff, center);%align axis by using point cloud density
        coeff = align3rdAxis(ptScene, coeff, center);
    
        % Build short vectors along principal axes for visualization
        [U,V,W] = makeUVWfromCoeff(coeff);
    
        % ---------------------------------------------------------------------
        % Package PCA results
        % ---------------------------------------------------------------------
        scene_pca.coeff = coeff;
        scene_pca.score = score;
        scene_pca.latent = latent;
        scene_pca.centroid = center;            % scene_pca.centroid = centroid;
        scene_pca.eulZYX = rotm2eul(coeff);
        scene_pca.UVW = [U,V,W];
        scene_pca_vec{idx} = scene_pca;
    
        % ---------------------------------------------------------------------
        % Compute in-plane orientation theta from the principal (major) axis +x
        % or [1 0 0] and the major axis of the object in an anti-clockwise direction
        %
        % - majorAxis: the first column of coeff, expressed in base_link
        % - theta: angle from +x toward majorAxis in XY, anti-clockwise, deg
        % ---------------------------------------------------------------------    
        majorAxis = [U(1), V(1), W(1)];    
        theta(idx) = atan2d(dot([0 0 1],...
                             cross([ 1 0 0],majorAxis)),...
                                   dot([ 1 0 0],majorAxis));
        if (theta(idx)<0)
            theta(idx) = 180 + theta(idx);
        end
        
        % ---------------------------------------------------------------------
        % Choose representative Z using a thin patch around (center.x, center.y)
        % (You kept "center" for xyz; leaving the patch code here for reference.)
        % ---------------------------------------------------------------------    
        roi = [centroid(1)-0.01 centroid(1)+0.01 centroid(2)-0.01 centroid(2)+0.01 -inf inf];
        indices = findPointsInROI(ptScene,roi);
        pt_surface_patch = select(ptScene,indices);
        z = mean(pt_surface_patch.Location(:,3));
        
        % Final representative position for this object
        xyz(idx,:) = [center(1) center(2) center(3)];
    end
end

%% Helper functions
% The following local functions are "docstringed" for students. They are
% internal utilities to stabilize PCA axis directions for reproducible angles.

function coeff_aligned = align2ndAxis(ptCloud, coeff, center)
% align2ndAxis  Consistently orients the PCA Y-axis using mass moments.
%
% Purpose
%   PCA eigenvectors can randomly flip signs across runs (both are valid
%   bases). This function tries to make the 2nd (Y) axis consistent: it
%   rotates the basis into the point cloud, measures squared "mass" above
%   and below that axis, and flips Y/Z if needed so that the positive side
%   tends to contain more mass. This reduces jitter in angles like theta.
%
% Inputs
%   ptCloud : pointCloud in base_link (same as used for PCA).
%   coeff   : 3x3 PCA basis (columns are principal axes).
%   center  : 1x3 reference point. We first translate points by -center to
%             make the axis test symmetric around the object's center.
%
% Output
%   coeff_aligned : 3x3 basis, possibly flipped along Y/Z for consistency.
%--------------------------------------------------------------------------
    
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
% align3rdAxis  Consistently orients the PCA Z-axis using mass moments.
%
% Purpose
%   Similar to align2ndAxis but targets the 3rd (Z) axis. We flip X/Z if
%   we detect more mass on the negative side to encourage a stable "up".
%
% Inputs
%   ptCloud : pointCloud in base_link (same as used for PCA).
%   coeff   : 3x3 PCA basis (columns are principal axes) after align2ndAxis.
%   center  : 1x3 reference point for centering before axis tests.
%
% Output
%   coeff_aligned : 3x3 basis, possibly flipped along X/Z for consistency.
%--------------------------------------------------------------------------

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
% makeUVWfromCoeff  Short vectors along PCA axes for plotting.
%
% Inputs
%   coeff : 3x3 PCA basis (columns are principal axes).
%
% Outputs
%   U,V,W : 3x1 vectors giving small arrow components along each axis.
%           These are handy for quiver3 plots at the object center.
%
% Example
%   [U,V,W] = makeUVWfromCoeff(coeff);
%   quiver3(cx,cy,cz, U(1),U(2),U(3)); % axis-1 arrow, etc.
%--------------------------------------------------------------------------
    
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





