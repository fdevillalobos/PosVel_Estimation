% Add additional inputs after sensor if you want to
% Example:
% your_input = 1;
% estimate_vel_handle = @(sensor) estimate_vel(sensor, your_input);
%
% We will only call estimate_vel_handle in the test function.
% Note that thise will only create a function handle, but not run the function



clc

% Camera Matrix (zero-indexed):
K = [314.1779       0       199.4848; ...
          0      314.2218   113.7838; ...
          0         0          1];

% Camera-IMU Calibration (see attached images for details):
Tb_to_c = [-0.04; 0.0; -0.03];
Yaw = pi/4;


%%
rows = 12;      cols = 9;
x_dim = 0.152;
y_dim = 0.152;
y_div = [0.152 0.152 0.178 0.152 0.152 0.178 0.152 0.152];
x_div = 0.152 * ones(11,1)';

Real = world(x_dim, y_dim, x_div, y_div, rows, cols);

%estimate_pose_handle = @(sensor) estimate_pose(sensor, K, Tb_to_c, Real);
estimate_vel_handle  = @(sensor) estimate_vel (sensor, K, Tb_to_c, Real);
%%
vel = zeros(3,size(data,2));
omg = zeros(3,size(data,2));
fprintf('Interpreting Data... \n');
for i = 1:size(data,2)
    %[pos(:,i), eul(:,i)] = estimate_pose_handle(data(i));
    [vel(:,i), omg(:,i)] = estimate_vel_handle(data(i));
end

%%
figure(2);
subplot(3,1,1); plot([data.t],omg(1,:)); hold on;
plot(time,vicon(10,:),'-r');
title('Omega vs time for dataset 1', 'FontSize', 18);

subplot(3,1,2); plot([data.t],omg(2,:)); hold on;
plot(time,vicon(11,:),'-r');

subplot(3,1,3); plot([data.t],omg(3,:)); hold on;
plot(time,vicon(12,:),'-r');

figure('units','normalized','outerposition',[0 0 1 1])
subplot(3,1,1); plot([data.t], vel(1,:)); hold on;
plot(time,vicon(7,:),'-r');
title('Velocities vs time for dataset 1', 'FontSize', 18);

subplot(3,1,2); plot([data.t], vel(2,:)); hold on;
plot(time,vicon(8,:),'-r');

subplot(3,1,3); plot([data.t], vel(3,:)); hold on;
plot(time,vicon(9,:),'-r');










