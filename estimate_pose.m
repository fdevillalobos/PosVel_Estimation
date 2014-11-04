function [pos, eul] = estimate_pose(data, K, Tb_to_c, Real)
%ESTIMATE_POSE 6DOF pose estimator based on apriltags
%   sensor - struct stored in provided dataset, fields include
%               - is_ready: logical, indicates whether sensor data is valid
%               - rpy, omg, acc: imu readings, you should not use these in this phase
%               - img: uint8, 240x376 grayscale image
%               - id: 1xn ids of detected tags
%               - p0, p1, p2, p3, p4: 2xn pixel position of center and
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
%              estimate_pose_handle = ...
%                  @(sensor) estimate_pose(sensor, your personal input arguments);
%   pos - 3x1 position of the quadrotor in world frame
%   eul - 3x1 euler angles of the quadrotor

%%
IDs = data.id;

if (size(IDs,2) > 0)
    
    p0c = data.p0';
    p1c = data.p1';
    p2c = data.p2';
    p3c = data.p3';
    p4c = data.p4';
    
    R_x_pi  =  [1      0         0;
                0   cos(pi)  -sin(pi);
                0   sin(pi)   cos(pi)];
    
    yaw_d = pi/4;
    
    R_z     =  [cos(yaw_d)  -sin(yaw_d)   0;
                sin(yaw_d)   cos(yaw_d)   0;
                   0             0        1];
    
    %%
    Points(:,:,1) = p1c;
    Points(:,:,2) = p2c;
    Points(:,:,3) = p3c;
    Points(:,:,4) = p4c;
    Points(:,:,5) = p0c;
    
    %%
    
    % Pre Allocate Matrix C with all the values for searching null space.
    C = zeros(2*size(IDs,1),9,4);
    for j = 1:size(data.id,2)
        idd = IDs(j) + 1;
        for k = 1:4
            u  = Points(j,1,k);
            X  = Real(idd,1,k);
            v  = Points(j,2,k);
            Y  = Real(idd,2,k);
            Cal_coord = K \ [u; v; 1];            % Inverse of K * uv1.
            x  = Cal_coord(1);
            y  = Cal_coord(2);
            C(2*j-1:2*j,:,k) = [-X,  0, x*X, -Y,  0, x*Y, -1,  0, x;
                0, -X, y*X,  0, -Y, y*Y,  0, -1, y];
        end
    end
    
    % 2N * 9 Matrix.
    Rel_Mat = [C(:,:,1); C(:,:,2); C(:,:,3); C(:,:,4)];
    
    [~,~,V]  = svd(Rel_Mat);           % Norm should be one. Checked!
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
    pos       =   ( Rw_to_c)' * (Tb_to_c - Tw_to_c);                              % Position Calculation.
    
    Rb_to_w =  R_x_pi * R_z * Rw_to_c;
    [phi,theta,psi] = RotToRPY_ZXY(Rb_to_w);
    eul = [phi;theta;psi];
    
else
    pos = [];
    eul = [];
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
