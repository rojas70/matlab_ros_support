function [xyz,theta,ptCloud_vec,scene_pca_vec] = findObjectPoses(ptCloud,rgbImage, bboxes, gridDownsample, nonPlaneMask)
%--------------------------------------------------------------------------
% findObjectPoses function is used to find the pose of the object using the
% bounding box output data and the point cloud data.
%
% Inputs:
% - ptCloud (pointCloud): original point cloud containing objects to be analyzed.
% - rgbImage (M×N×3 uint8 array): RGB image corresponding to the point cloud, used for generating masks.
% - bboxes (P×4 array): bounding boxes from YOLO detector; each row [x, y, width, height] represents a bounding box.
% - gridDownsample (double): grid size for downsampling the point cloud (e.g., 0.002 m).
% - nonPlaneMask (optional, logical array): binary mask of non-plane points (objects); same size as rgbImage. Defaults to all ones if omitted.
%
% Outputs:
% - xyz (P×3 array):   XYZ centroid positions for each object.
% - theta (1×P array): Orientations of each object (angle in degrees from the positive x-axis).
% - ptCloud_vec (cell array of pointCloud): Point cloud segments corresponding to each individual object.
% - scene_pca_vec (cell array of structs): PCA analysis data for each object including principal axes, centroid, eigenvalues (latent), and Euler angles.
% 
% Method overview
% 1. Init variables
% 2. Process Each Detected Object
%   - Generate object mask
%   - Extract and Downsample Points
%   - Segment Points into clusters
%   - Perform PCA Analysis
%   - Store PCA Results
%   - Calculate Object Orientation
%   - Determine Object Position
%--------------------------------------------------------------------------
%   Copyright 2023 The MathWorks, Inc.
%--------------------------------------------------------------------------

    % Check if nonPlaneMask argument is provided; if not, default to all ones (no masking)
    if (nargin<5)
        nonPlaneMask = ones(size(rgbImage,1),size(rgbImage,2));
    end
    
    % Initialize output arrays and cell arrays for objects
    numObjects      = size(bboxes, 1);              % Number of detected objects
    ptCloud_vec     = cell(numObjects, 1);          % Cell array to store object point clouds
    scene_pca_vec   = cell(numObjects, 1);          % Cell array to store PCA results
    theta           = [];                           % Initialize array for orientation angles
    xyz             = [];                           % Initialize array for object positions

    % Loop through each detected object based on bounding boxes
    for idx= 1:numObjects
    
        % Create initial binary mask based on bounding box
        initMask = zeros(size(rgbImage,1),size(rgbImage,2));
        roi = int32(round(bboxes(idx, :)));                     % Get bounding box as integer coordinates

        % Extract region of interest (ROI) from bounding box, clamped to image boundaries
        tr = max(roi(2), 0);                                   % Top row
        ur = min(roi(2) + roi(4), size(rgbImage,1));           % Bottom row
        lc = max(roi(1), 0);                                   % Left column
        rc = min(roi(1) + roi(3), size(rgbImage,2));           % Right column        
        
       
        % Mark ROI pixels in initial mask by setting those index entries to 1       
        initMask(tr:ur, lc:rc) = 1;

        % Combine initial mask with non-plane mask
        instanceMask = initMask(:) & nonPlaneMask;% & ROIMask;
    
        % Select points from the point cloud using the final mask
        pt_instance = select(ptCloud, find(instanceMask));

        % Downsample point cloud for efficiency and uniformity
        pt_instance = pcdownsample(pt_instance, 'gridAverage', gridDownsample);

        % Segment the downsampled point cloud into clusters based on distance: pcsegdist(ptCloud,minDistance)
        [lab_instance, num_instance] = pcsegdist(pt_instance, gridDownsample * sqrt(3));
    
        % Determine the largest cluster, assuming it is the primary object
        c = [];
        for j = 1:num_instance
            c(j) = nnz(lab_instance == j);      % Count number of points in each cluster
        end

        % Find cluster with the maximum points
        [~, max_labelJ] = max(c);
    
        % Extract point cloud corresponding to the largest cluster
        ptCloud_vec{idx} = select(pt_instance, find(lab_instance==max_labelJ));
        ptScene = ptCloud_vec{idx};
        
        % Perform PCA (Principal Component Analysis) on object point cloud
        [coeff, score, latent] = pca(ptScene.Location);
    
        % Ensure PCA axes form a right-handed coordinate system
        if (det(coeff) < 0)  % det = -1 means rigyht-handed system, so transforms into left-handed system
            coeff(:, 2) = -coeff(:, 2);
        end
    
        % Align PCA axes based on spatial distribution of points
        centroid = mean(ptScene.Location);                   % Compute centroid of points
        coeff    = align2ndAxis(ptScene, coeff, centroid);   % Align second PCA axis
        coeff    = align3rdAxis(ptScene, coeff, centroid);   % Align third PCA axis
        
        % Prepare PCA vectors for visualization purposes
        [U,V,W] = makeUVWfromCoeff(coeff);
    
        % Store PCA results in a structured format
        scene_pca.coeff     = coeff;
        scene_pca.score     = score;
        scene_pca.latent    = latent;
        scene_pca.centroid  = centroid;
        scene_pca.eulZYX    = rotm2eul(coeff);
        scene_pca.UVW       = [U,V,W];
        scene_pca_vec{idx}  = scene_pca;

        % Determine orientation angle relative to positive X-axis
        majorAxis = [U(1), V(1), W(1)];
    
        % This calculates the angle between the positive x-axis ([ 1 0 0]) and the major axis of the object in an anti-clockwise direction
        theta(idx) = atan2d(dot([0 0 1], cross([ 1 0 0],majorAxis)), dot([ 1 0 0],majorAxis));

        % Adjust angle to ensure it is within [0,180] degrees
        if (theta(idx)<0)
            theta(idx) = 180 + theta(idx);
        end
    
        % Determine precise XYZ position, refining Z using points close to centroid
        roi = [centroid(1)-0.01 centroid(1)+0.01 centroid(2)-0.01 centroid(2)+0.01 -inf inf];
        indices = findPointsInROI(ptScene,roi);
        
        pt_surface_patch = select(ptScene,indices);
        z = mean(pt_surface_patch.Location(:,3));

        % Store XYZ position of the object
        xyz(idx,:) = [centroid(1) centroid(2) z];
    end
end

%--------------------------------------------------------------------------
% Helper functions
%--------------------------------------------------------------------------
function coeff_aligned = align2ndAxis(ptCloud, coeff, center)
%--------------------------------------------------------------------------
% Function: align2ndAxis
%
% Aligns the second PCA axis based on point density distribution along the
% Y-axis. Ensures the second axis points toward the direction with higher
% point density.
%
% Inputs:
% - ptCloud : Input point cloud of the object.
% - coeff   : Initial PCA coefficient matrix.
% - center  : Centroid of the point cloud.
%
% Output:
% - coeff_aligned : Aligned PCA coefficient matrix ensuring correct Y-axis
%                   orientation.
%--------------------------------------------------------------------------

    % Shift point cloud so centroid is at the origin for analysis
    ptCloud_aligned = pctransform(ptCloud, rigid3d(eye(3), -center));

    % Rotate point cloud based on initial PCA coefficients for alignment
    ptCloud_aligned = pctransform(ptCloud_aligned, rigid3d(coeff, [0 0 0]));

    % Extract Y-coordinates of transformed points for horizontal analysis
    points_y = ptCloud_aligned.Location(:, 2);

    % Separate points into those on positive and negative sides of Y-axis
    posy = points_y > 0;

    % Compute squared distances of points on positive and negative sides
    moments_pos = sum(points_y(posy) .^ 2);    % Positive side
    moments_neg = sum(points_y(~posy) .^ 2);   % Negative side

    % Prepare an alternate PCA coefficient matrix with inverted axes
    coeff_rot = coeff;
    coeff_rot(:, 2) = -coeff(:, 2);    % Flip second principal axis
    coeff_rot(:, 3) = -coeff(:, 3);    % Flip third principal axis

    % Determine alignment based on which side has greater point density
    if (moments_pos > moments_neg)
        % Keep original orientation if positive side dominates
        coeff_aligned = coeff;
    else
        % Invert orientation if negative side dominates
        coeff_aligned = coeff_rot;
    end
end


%--------------------------------------------------------------------------
function coeff_aligned = align3rdAxis(ptCloud, coeff, center)
%--------------------------------------------------------------------------
% Function: align3rdAxis
%
% Aligns the third PCA axis (typically the vertical Z-axis) based on point
% density along the vertical direction. Ensures consistent vertical
% orientation for the object's pose.
%
% Inputs:
% - ptCloud : Input point cloud of the object.
% - coeff   : Initial PCA coefficient matrix.
% - center  : Centroid of the point cloud.
%
% Output:
% - coeff_aligned : Aligned PCA coefficient matrix ensuring correct vertical
%                   orientation.
%--------------------------------------------------------------------------

    % Shift point cloud so centroid is at the origin for analysis
    ptCloud_aligned = pctransform(ptCloud, rigid3d(eye(3), -center));

    % Rotate point cloud based on initial PCA coefficients for alignment
    ptCloud_aligned = pctransform(ptCloud_aligned, rigid3d(coeff, [0 0 0]));

    % Extract Z-coordinates of transformed points for vertical analysis
    points_z = ptCloud_aligned.Location(:, 3);

    % Separate points into those above and below the horizontal plane (Z = 0)
    posy = points_z > 0;

    % Compute squared distances of points above and below the horizontal plane
    moments_pos = sum(points_z(posy) .^ 2);    % Above plane
    moments_neg = sum(points_z(~posy) .^ 2);   % Below plane

    % Prepare an alternate PCA coefficient matrix with inverted axes
    coeff_rot = coeff;
    coeff_rot(:, 1) = -coeff(:, 1);    % Flip first principal axis
    coeff_rot(:, 3) = -coeff(:, 3);    % Flip third principal axis (vertical axis)

    % Determine alignment based on which side has greater point density
    if (moments_pos > moments_neg)
        % Keep original orientation if points above plane dominate
        coeff_aligned = coeff;
    else
        % Invert orientation if points below plane dominate
        coeff_aligned = coeff_rot;
    end
end


%--------------------------------------------------------------------------
function [U,V,W] = makeUVWfromCoeff(coeff)
%--------------------------------------------------------------------------
% Converts PCA coefficients into scaled 3D vectors for visualizing PCA axes.
%
% In point cloud visualization (pcshow), the axes you're plotting (with
% quiver3) are overlayed onto points which are typically very close to each
% other, often within millimeter-level distances.
%
% Multiplying by a small number (like 0.05) "scales" the PCA axis to a size 
% that fits nicely within the local dimensions of your visualized object. 
% It "extends" clarity rather than actual length, effectively adjusting it 
% from "way too long" to "just right" for visualization purposes.
%--------------------------------------------------------------------------
    
    % Scale down to match point cloud density
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
