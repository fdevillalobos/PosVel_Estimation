function [New_valid, inlier_ind] = ransac_est(All_cDot, K, Tw_to_c, Rw_to_c, Z_Matrix, thresh)

numVect = 1:size(All_cDot,2);                                               % vector that goes [1 2 3 4 ... size(pDot)
max_count = 0;                                                              % Initialize max_count
B    = zeros(6,6);
C    = zeros(size(All_cDot,2)-6,6);
for i=1:1000
    % Get 3 random row numbers between 1 and the size or the vectors.
    randsel = randperm(size(All_cDot,2),3);
    pDot_Sample = All_cDot(:,randsel');
    pDot_Comple = numVect;
    pDot_Comple(randsel) = [];                                              % Creates a vector that has every number from 1:size(pDot) exept for the three selected.
    velM_Sample = pDot_Sample(:);    
    
    for j = 1:3
        %velM(2*j-1:2*j) = pDot_Sample(j,:)';
        
        Aux_1           = (Z_matrix(randsel',1)/norm(Z_matrix(randsel',1)));
        z               = -(Tw_to_c(3)) / Aux_1(3);
        
        B(2*j-1:2*j,:) = [-1/z,     0,  pDot_Sample(j,1)/z,  pDot_Sample(j,1)*pDot_Sample(j,2),            -(1+pDot_Sample(j,1)^2),    pDot_Sample(j,2);
                             0,  -1/z,  pDot_Sample(j,2)/z,               1+pDot_Sample(j,2)^2, -pDot_Sample(j,1)*pDot_Sample(j,2),   -pDot_Sample(j,1)];
    end

    
    
 
    
    
    
    %Get the difference between the Homography calculated points and the found ones.
    X_diff = (X_Hmp - Corr_Points_X2);
    Y_diff = (Y_Hmp - Corr_Points_Y2);
    % Calculate the distance between the calculated points and the real ones.
    dist = sqrt(X_diff.^2 + Y_diff.^2);
    % Count how many of them are actually smaller than the threshold.
    thresh_dist = dist < thresh;
    % Store the value and vector for the maximum count.
    if (sum(thresh_dist) > max_count)
        max_count = sum(thresh_dist);
        voting_vector = thresh_dist;
    end
end

% Take the Values of the ones that voted for the ones with more votes.
% X1_valid = Corr_Points_X1(find(voting_vector == 1));
% Y1_valid = Corr_Points_Y1(find(voting_vector == 1));
% X2_valid = Corr_Points_X2(find(voting_vector == 1));
% Y2_valid = Corr_Points_Y2(find(voting_vector == 1));
% % Calculate the new more accurate Homography estimation.
% H = est_homography(X2_valid, Y2_valid, X1_valid, Y1_valid);

in_index = 0;
inlier_ind = zeros(max_count,1);
for i = 1:size(voting_vector,1)
    if(voting_vector(i) == 1)
        in_index = in_index + 1;
        inlier_ind(in_index) = i;
    end
end

Old_valid = [Corr_Points_X1(voting_vector == 1), Corr_Points_Y1(voting_vector == 1)];
New_valid = [Corr_Points_X2(voting_vector == 1), Corr_Points_Y2(voting_vector == 1)];

end



    
    