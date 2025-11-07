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

    % Group
    U = [U1;U2;U3];
    V = [V1;V2;V3];
    W = [W1;W2;W3];

end