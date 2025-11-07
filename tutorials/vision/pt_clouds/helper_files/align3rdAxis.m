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
