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
