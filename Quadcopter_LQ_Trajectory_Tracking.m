clc; clear; close all; 
format long;

tic % Start timer for computation

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% LQ Controller for Quadcopter Trajectory Tracking %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Platform Parameters
m = 1.25; % Mass (kg)
g = 9.81; % Gravity (m/s^2)
Ixx = 0.0232; % Inertia (kg m^2)
Iyy = 0.0232; % Inertia (kg m^2)
Izz = 0.0468; % Inertia (kg m^2)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% Trajectories %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Uncomment trajectory of choice

%%%% 3D LINE %%%%
% dt = 0.001; % Time step (s)
% tf = 10; % Final time (s)
% t = 0:dt:tf; % Time array (s)
% N = length(t); % Number of time steps
% 
% x_0 = [0; 0; 0]; % Starting point (x,y,z)
% x_f = [-10; 10; 10]; % Final point (x,y,z)
% 
% traj = x_0 + (x_f - x_0) * (t / tf);  % Positional trajectory
% traj_vel = repmat((x_f - x_0) / tf, 1, N); % Derivative for velocity
% 
% % Generate 12 state reference trajectory
% x_ref = zeros(12, N); % [x; dx; y; dy; z; dz; phi; dphi; theta; dtheta; psi; dpsi]
% x_ref(1, :) = traj(1, :);
% x_ref(2, :) = traj_vel(1, :);
% x_ref(3, :) = traj(2, :);
% x_ref(4, :) = traj_vel(2, :);
% x_ref(5, :) = traj(3, :);
% x_ref(6, :) = traj_vel(3, :);
% 
% % Initial conditions - [x, dx, y, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi]
% state = zeros(12,N); % Tracks state over time
% state(:,1) = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];

%%%% UPWARDS SPIRAL %%%%
% dt = 0.001; % Time step (s)
% tf = 30; % Final time (s)
% t = 0:dt:tf; % Time array (s)
% N = length(t); % Number of time steps
% 
% r = 5; % Radius of spiral (m)
% w = 2 * pi / (tf/3); % Angular velocity (rad/s)
% linearRate = 0.75; % Linear rate of climb (m/s)
% 
% traj_x = r * cos(w*t); % X position trajectory
% traj_y = r * sin(w*t); % Y position trajectory
% traj_z = linearRate * t; % Z position trajectory
% 
% vel_x = -r * w * sin(w*t); % X velocity trajectory
% vel_y = r * w * cos(w*t); % Y velocity trajectory
% vel_z = linearRate * ones(1,N); % Z velocity trajectory
% 
% % Generate 12 state reference trajectory
% x_ref = zeros(12, N); % [x; dx; y; dy; z; dz; phi; dphi; theta; dtheta; psi; dpsi]
% x_ref(1, :) = traj_x;
% x_ref(2, :) = vel_x;
% x_ref(3, :) = traj_y;
% x_ref(4, :) = vel_y;
% x_ref(5, :) = traj_z;
% x_ref(6, :) = vel_z;
% 
% % Initial conditions - [x, dx, y, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi]
% state = zeros(12,N); % Tracks state over time
% state(:,1) = [5; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];

%%%% Rose Petal %%%%
dt = 0.001; % Time step (s)
tf = 30; % Final time (s)
t = 0:dt:tf; % Time array (s)
N = length(t); % Number of time steps

r = 5; % Radius of path (m)
w = 2 * pi / tf; % angular velocity (rad/s)
k = 2; % Number of loops | odd k = k 'petals' | even k = 2 *k 'petals'

traj_x = r * cos(k*w*t) .* cos(w*t); % X velocity trajectory
traj_y = r * cos(k*w*t) .* sin(w*t); % Y velocity trajectory
traj_z = (r/4) * sin(w*t); % Z velocity trajectory

vel_x = gradient(traj_x, dt); % X velocity trajectory
vel_y = gradient(traj_y, dt); % Y velocity trajectory
vel_z = gradient(traj_z, dt); % Z velocity trajectory

% Generate 12 state reference trajectory
x_ref = zeros(12, N); % [x; dx; y; dy; z; dz; phi; dphi; theta; dtheta; psi; dpsi]
x_ref(1, :) = traj_x;
x_ref(2, :) = vel_x;
x_ref(3, :) = traj_y;
x_ref(4, :) = vel_y;
x_ref(5, :) = traj_z;
x_ref(6, :) = vel_z;

% Initial conditions - [x, dx, y, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi]
state = zeros(12,N); % Tracks state over time
state(:,1) = [5; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];

%%%% Reference Inputs %%%%
% Assume desired state near hover at each time step for slower, controlled
% trajectories
U_ref = [m*g; 0; 0; 0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% WIND NOISE %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% At the end of this section, uncomment the wind acceleration component
% being all zeros to eliminate wind noise
% Wind speed noise is calculated using exponential smoothing to
% smooth wind values generated from a Guassian distribution

vel_mean= 5; % Mean wind speed (m/s)
vel_std = 2; % Wind speed standard deviation

ang_mean = deg2rad(25); % Mean direction of wind with respect to x-direction (rad)
ang_std = deg2rad(5); % Wind direction standard deviation

Vel = zeros(1,N); % Store wind velocity data
Vel_x = zeros(1,N); % Store x-direction wind velocity data
Vel_y = zeros(1,N); % Store y-direction wind velocity data
Dir = zeros(1,N); % Store wind direction data

alpha = 0.99; % Smoothing parameter
vel_noise = 0; % Initial wind speed noise
ang_noise = 0; % Initial wind speed noise

for h = 1:N
    % Generate wind speed and angle noise with exponential smoothing and 
    % randn() Gaussian distribution
    vel_noise = alpha * vel_noise + sqrt(1 - alpha^2) * vel_std * randn();
    ang_noise = alpha * ang_noise + sqrt(1 - alpha^2) * ang_std * randn();

    % Calculate wind speed and angle value with respect to mean value
    vel_abs = vel_mean + vel_noise;
    ang_abs = ang_mean + ang_noise;

    % Calculate and store x- and y-components
    Vel_x(h) = vel_abs * cos(ang_abs);
    Vel_y(h) = vel_abs * sin(ang_abs);

    % Store absolute values for visualization
    Vel(h) = vel_abs;
    Dir(h) = ang_abs;
end

% Combine x and y wind states
V = zeros(12,N); 
V(1,:) = Vel_x;
V(3,:) = Vel_y;

% Calculate acceleration due to drag
Cd = 0.75; % Drag coefficient estimate
A = 0.1; % Area estimate (m^2)
p = 1.225; % Air density (kg/m^3)
S = (1/2) * p * A * Cd; % Placeholder for 0.5*p*A*Cd in drag equation
F_wind = S .* V.^2; % Force due to wind

% a_wind = F_wind ./ m; % Acceleration due to wind if wind noise is desired

a_wind = zeros(12,N); % Acceleration due to wind if no wind noise is desired

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% Q & R COSTS  %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Combined costs from GA optimization
QR = [449, 161, 392, 94.6, 486, 58.2, 13.1, 1.91, 310, 2.2, 297, 135, 0.104, 0.139, 0.033, 8.86];

% Split out into Q and R costs
Q = diag(QR(1:12));
R = diag(QR(13:16));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% STATE SPACE MODEL & GAINS %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

A = zeros(12); % Initialize A-matrix
B = zeros(12, 4); % Initialize B-matrix

% Populate A-matrix
A(1,2) = 1;
A(2,9) = g;
A(3,4) = 1;
A(4,7) = -g;
A(5, 6) = 1;
A(7,8) = 1;
A(9, 10) = 1;
A(11, 12) = 1;

% Populate B-matrix
B(6, 1) = 1/m;
B(8, 2) = 1/Ixx;
B(10,3) = 1/Iyy;
B(12,4) = 1/Izz;

% Compute Gain
[K,S,P] = lqr(A,B,Q,R);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% INITIALIZE VARIABLES %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

U_hist = zeros(4,N); % Tracks control inputs over time

% Store positional errors over time for performance analysis
e_x = zeros(1,N);
e_y = zeros(1,N);
e_z = zeros(1,N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% SIMULATION %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i = 1:N-1

    % State error
    e = state(:,i) - x_ref(:,i);

    % Calculate system control input from state error
    U = U_ref - K * e;

    % Limit inputs to operational ranges
    U(1) = min(max(U(1), -m*g), 25 - m*g); % limit between free fall (0 N thrust) and +25 N
    U(2) = min(max(U(2), -7), 7);
    U(3) = min(max(U(3), -7), 7);
    U(4) = min(max(U(4), -7), 7);

    % Track positional error over time - store up for RMSE calculation
    e_x(i) = e(1);
    e_y(i) = e(3);
    e_z(i) = e(5);

    % Calculate state change at time step
    dx = A * state(:,i) + B * U + a_wind(:,i);
    dx(6) = dx(6) - g; % Account for gravity not represented in linearized state space form

    % Calculate updated state
    state(:,i+1) = state(:,i) + dx * dt;

    % Log inputs
    U(1) = U(1) + m*g; % Account for gravity again after calculation so plots look correct
    U_hist(:,i) = U;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% Performance Calculation %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

compTime = toc; % End timer for computation
fprintf('\nComputation Time: %.3f seconds\n', compTime)

% Root Mean Square Error (RMSE) for positional states x, y, and z
rmse_3D = sqrt(mean(e_x.^2 + e_y.^2 + e_z.^2));
rmse_x = sqrt(mean(e_x.^2));
rmse_y = sqrt(mean(e_y.^2));
rmse_z = sqrt(mean(e_z.^2));

fprintf('\nRoot Mean Squared Error for LQ Trajectory Tracker\n')
fprintf('- - - - - - - - - - - - - - -\n');
fprintf('3-Dimensional RMSE: %.3f meters\n', rmse_3D)
fprintf('X-Position RMSE: %.3f meters\n', rmse_x)
fprintf('Y-Position RMSE: %.3f meters\n', rmse_y)
fprintf('Z-Position RMSE: %.3f meters\n', rmse_z)

% Calculate oscillations as a local minima/maxima
% Calculate 1-D velocities
dx = gradient(state(1,:), t);
dy = gradient(state(3,:), t);
dz = gradient(state(5,:), t);

% Calculate 1-D accelerations
ddx = gradient(dx, t);
ddy = gradient(dy, t);
ddz = gradient(dz, t);

% Combine into arrays for manipulation
d = [dx; dy; dz];
dd = [ddx; ddy; ddz];

% Take the cross product and calculate the curvature
vCrossa = cross(d', dd')';
K = sqrt(sum(vCrossa.^2, 1)) ./ (sqrt(dx.^2 + dy.^2 + dz.^2).^3);

% Threshold curvature to find number of sharp turns indicating a lack of
% smoothness
[sharp_turns, ~] = findpeaks(K, 'MinPeakHeight', 0.75);
sharp_turns = length(sharp_turns);
fprintf('\n\nLocal Minima/Maxima Smoothness Analysis\n')
fprintf('- - - - - - - - - - - - - - -\n')
fprintf('Number of sharp turns: %.0f\n', sharp_turns)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% Visualization %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Plot colors
colors = [0.8500 0.3250 0.0980; % 'o'
        0.4660 0.6740 0.1880; % 'g'
        0.3010 0.7450 0.9330; % 'c'
        0.4940 0.1840 0.5560]; % 'm'

% Plot states
figure;
labels = {'x (m)', 'y (m)', 'z (m)', '\phi (rad)', '\theta (rad)', '\psi (rad)'};
ind = [1, 3, 5, 7, 9, 11];
for j = 1:6
    subplot(3,2,j);
    plot(t, state(ind(j),:), 'b', 'LineWidth', 1.3);
    hold on;
    plot(t, x_ref(ind(j), :), 'k--', 'LineWidth', 1.1);
    ylabel(labels{j});
    xlabel('Time (s)');
    legend('Actual','Reference');
    grid on;
end
sgtitle('Quadcopter States with LQ Controller');

% Plot inputs
figure;
labels = {'U1 - Thrust (N)', 'U2 - Roll Torque (Nm)', 'U3 - Pitch Torque (Nm)', 'U4 - Yaw Torque (Nm)'};
for k = 1:4
    subplot(2,2,k);
    plot(t, U_hist(k,:), 'Color', colors(k,:), 'LineWidth', 1.4);
    xlabel('Time (s)');
    ylabel(labels{k});
    grid on;
end
sgtitle('Quadcopter Control Inputs with LQ Controller')

% Plot 3D trajectory
figure;
plot3(x_ref(1,:), x_ref(3,:), x_ref(5,:), 'k--', 'LineWidth', 1.5);
hold on;
plot3(state(1,:), state(3,:), state(5,:), 'b', 'LineWidth', 2);
hold on;
scatter3(x_ref(1,1), x_ref(3,1), x_ref(5,1), 75, 'go', 'filled');
hold on;
scatter3(x_ref(1,N), x_ref(3,N), x_ref(5,N), 75, 'ro', 'filled');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
legend('Reference Trajectory','Actual Trajectory','Start','End');
grid on;
axis equal;
title('3D Position for Trajectory Tracking with LQ Controller');