function [vel, omg] = estimate_vel(data, K, Tb_to_c, Real)
%ESTIMATE_VEL 6DOF velocity estimator
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - t: timestamp
%          - rpy, omg, acc: imu readings, you should not use these in this phase
%          - img: uint8, 240x376 grayscale image
%          - id: 1xn ids of detected tags
%          - p0, p1, p2, p3, p4: 2xn pixel position of center and
%                                four corners of detected tags
%            Y
%            ^ P3 == P2
%            | || P0 ||
%            | P4 == P1
%            o---------> X
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              estimate_vel_handle = ...
%                  @(sensor) estimate_vel(sensor, your personal input arguments);
%   vel - 3x1 velocity of the quadrotor in world frame
%   omg - 3x1 angular velocity of the quadrotor

%%
persistent img_old corn_pix_old time_old dt_estimate
alpha = 0.1;
IDs = data.id;

% dt_estimate calculation. Time between frames.
if(isempty(time_old))
    dt_estimate = 0.02;
else
    dt_estimate = dt_estimate*(1-alpha) + alpha*(data.t-time_old);
end

% IF there are No IDs, then just return the data as empty.
if(isempty(IDs))
    vel = [];
    omg = [];
    time_old = data.t;
    img_old  = [];
    corn_pix_old = [];
    return;
end


% Execute this ONCE every time it recovers sight of all the April tags.
if(isempty(img_old))
    img_old = data.img;
    corn_pix_old = corner(img_old,'MinimumEigenvalue',100);
    time_old  = data.t;
    vel = zeros(3,1);
    omg = zeros(3,1);
else
    
    %%
    %******************************************************************%
    %************************** Phase I Code **************************%
    %******************************************************************%
    
    if (size(IDs,2) > 0)
        
        Points(:,:,1) = data.p1';
        Points(:,:,2) = data.p2';
        Points(:,:,3) = data.p3';
        Points(:,:,4) = data.p4';
        Points(:,:,5) = data.p0';
        
        % Pre Allocate Matrix C with all the values for searching null space.
        C = zeros(2*size(IDs,1),9,4);
        for j = 1:size(data.id,2)
            idd = IDs(j) + 1;
            for k = 1:4
                %u  = Points(j,1,k);
                %X  = Real(idd,1,k);
                %v  = Points(j,2,k);
                %Y  = Real(idd,2,k);
                Cal_coord = K \ [Points(j,1,k); Points(j,2,k); 1];          % Inverse of K * uv1.
                %x  = Cal_coord(1);
                %y  = Cal_coord(2);
                C(2*j-1:2*j,:,k) = [-Real(idd,1,k),  0, Cal_coord(1)*Real(idd,1,k), -Real(idd,2,k),  0, Cal_coord(1)*Real(idd,2,k), -1,  0, Cal_coord(1);
                                     0, -Real(idd,1,k), Cal_coord(2)*Real(idd,1,k),  0, -Real(idd,2,k), Cal_coord(2)*Real(idd,2,k),  0, -1, Cal_coord(2)];
            end
        end
        
        % 2N * 9 Matrix.
        %Rel_Mat = [C(:,:,1); C(:,:,2); C(:,:,3); C(:,:,4)];
        
        [~,~,V]  = svd([C(:,:,1); C(:,:,2); C(:,:,3); C(:,:,4)]);           % Norm should be one. Checked!
        A = reshape(V(:,end),3,3);
        A = A / norm(A(:,1));
        if(A(3,3) < 0)
            A = -A;
        end
        
        [U,~,V]  = svd([A(:,1) A(:,2) cross(A(:,1),A(:,2))]);
        
        S = [1   0      0;
             0   1      0;
             0   0   det(U*V')];
        
        Rw_to_c =  U*S*V';
        
        Tw_to_c =   A(:,3);
%         pos       =   ( Rw_to_c)' * (Tb_to_c - Tw_to_c);                  % Position Calculation.
%         
%         Rb_to_w =  R_x_pi * R_z * Rw_to_c;
%         [phi,theta,psi] = RotToRPY_ZXY(Rb_to_w);
%         eul = [phi;theta;psi];
    end
    
    %******************** END OF PHASE I CODE ***************************%
    %********************************************************************%
    
    %% For every other loop except for the first one.
    pointTracker = vision.PointTracker;                                     % Track old Corners in the New Image.
    initialize(pointTracker,corn_pix_old,img_old);
    [tracked_old_corn,point_validity] = step(pointTracker,data.img);
    
    Corner_New = K\[tracked_old_corn'; ones(1,size(tracked_old_corn,1))];   % Pixel Position of the Old Corners in the NEW Photo Frame. Corner_New = (x,y). tracked = (u,v).
    %Corner_New = bsxfun(@rdivide,Corner_New, Corner_New(3,:));
    Corner_New = Corner_New(1:2,point_validity)';                           % Remove row of ones.
    
    Corner_old = K\[corn_pix_old'; ones(1,size(corn_pix_old,1))];           % Convert (u,v) to (x,y).
    %Corner_old = bsxfun(@rdivide,Corner_old, Corner_old(3,:));
    Corner_old = Corner_old(1:2,point_validity)';
    
    
    Corner_dot =  ((Corner_New - Corner_old) / dt_estimate)';               % Delta of position for detected corners. delta_p / dt = vel.
    
%     Corner_Pixel_Old = corn_pix_old(point_validity,:);
%     figure(10);
%     imshow(data.img); hold on;
%     quiver(Corner_Pixel_Old(:,1), Corner_Pixel_Old(:,2), Corner_dot(:,1), Corner_dot(:,2));
    
    Z_matrix   = Rw_to_c' * [Corner_New'; ones(1,size(Corner_New,1))];
    
    B    = zeros(2*size(Corner_New,1),6);
    velM = Corner_dot(:);
    
    for j = 1:size(Corner_New,1)
        Aux_1           = (Z_matrix(:,1)/norm(Z_matrix(:,1)));
        z               = -(Tw_to_c(3)) / Aux_1(3);
        
        B(2*j-1:2*j,:)  = [-1/z,     0,  Corner_New(j,1)/z,  Corner_New(j,1)*Corner_New(j,2),            -(1+Corner_New(j,1)^2),    Corner_New(j,2);
                              0,  -1/z,  Corner_New(j,2)/z,               1+Corner_New(j,2)^2, -Corner_New(j,1)*Corner_New(j,2),   -Corner_New(j,1)];
    end
    
    %CorNew_Val = Corner_New;
    %[CorDot_Valid, inlier_ind] = ransac_est_homography(Corner_dot, K, Tw2, Rw_to_c, 0.004);
    
    Velocities = B\velM;
    
%     R_x_pi  =  [1      0         0;
%                 0   cos(pi)  -sin(pi);
%                 0   sin(pi)   cos(pi)];
%         
%     yaw_d = pi/4;
%         
%     R_z     =  [cos(yaw_d)  -sin(yaw_d)   0;
%                 sin(yaw_d)   cos(yaw_d)   0;
%                 0             0        1];
     
    img_old = data.img;
    corn_pix_old = corner(data.img,'MinimumEigenvalue',100);
    time_old  = data.t;
    vel = Rw_to_c' * Velocities(1:3);
    omg = Rw_to_c' * Velocities(4:6);
    
    
end

end










function [phi,theta,psi] = RotToRPY_ZXY(R)
%RotToRPY_ZXY Extract Roll, Pitch, Yaw from a world-to-body Rotation Matrix
%   The rotation matrix in this function is world to body [bRw] you will
%   need to transpose the matrix if you have a body to world [wRb] such
%   that [wP] = [wRb] * [bP], where [bP] is a point in the body frame and
%   [wP] is a point in the world frame
%   written by Daniel Mellinger
%   bRw = [ cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta),
%           cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta),
%          -cos(phi)*sin(theta)]
%         [-cos(phi)*sin(psi), cos(phi)*cos(psi), sin(phi)]
%         [ cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi),
%           sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi),
%           cos(phi)*cos(theta)]

phi = asin(R(2,3));
psi = atan2(-R(2,1)/cos(phi),R(2,2)/cos(phi));
theta = atan2(-R(1,3)/cos(phi),R(3,3)/cos(phi));

end