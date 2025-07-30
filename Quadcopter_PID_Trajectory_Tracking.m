clc; clear; close all; 
format long;

tic % Start timer for computation

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% PID Controller for Quadcopter Trajectory Tracking %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
dt = 0.01; % Time step (s)
tf = 10; % Final time (s)
t = 0:dt:tf; % Time array (s)
N = length(t); % Number of time steps

x_0 = [0; 0; 0]; % Starting point (x,y,z)
x_f = [-10; 10; 10]; % Final point (x,y,z)

traj_pos = x_0 + (x_f - x_0) * (t / tf); % Positional trajectory
traj_vel = repmat((x_f - x_0) / tf, 1, N); % Derivative for velocity
traj_accel = [0; 0; 0] * t; % Acceleration trajectory
traj_accel(3,:) = traj_accel(3,:) + 9.81; % Add gravity to z-trajectory

psiDes = 0; % Constant yaw value

% Initial conditions - [x, dx, y, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi]
state = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];

%%%% UPWARDS SPIRAL %%%%
% dt = 0.01; % Time step (s)
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
% traj_pos = [traj_x; % Combined (x,y,z) trajectory
%             traj_y;
%             traj_z];
% 
% vel_x = -r * w * sin(w*t); % X velocity trajectory
% vel_y = r * w * cos(w*t); % Y velocity trajectory
% vel_z = linearRate * ones(1,N); % Z velocity trajectory
% traj_vel = [vel_x; % Combined velocity trajectory
%             vel_y;
%             vel_z];
% 
% accel_x = -r * w^2 * cos(w*t); % X acceleration trajectory
% accel_y = -r * w^2 * sin(w*t); % Y acceleration trajectory
% accel_z = 0 * t; % Z acceleration trajectory
% traj_accel = [accel_x; % Combined acceleration trajectory
%               accel_y;
%               accel_z];
% traj_accel(3,:) = traj_accel(3,:) + 9.81; % Add gravity to Z-component
% 
% psiDes = 0; % Constant yaw value
% 
% % Initial conditions - [x, dx, y, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi]
% state = [5; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];

%%%% Rose Petal %%%%
% dt = 0.01; % Time step (s)
% tf = 30; % Final time (s)
% t = 0:dt:tf; % Time array (s)
% N = length(t); % Number of time steps
% 
% r = 5; % Radius of path (m)
% w = 2 * pi / tf; % angular velocity (rad/s)
% k = 2; % Number of loops | odd k = k 'petals' | even k = 2 *k 'petals'
% 
% traj_x = r * cos(k*w*t) .* cos(w*t); % X velocity trajectory
% traj_y = r * cos(k*w*t) .* sin(w*t); % Y velocity trajectory
% traj_z = (r/4) * sin(w*t); % Z velocity trajectory
% traj_pos = [traj_x; % Combined (x,y,z) trajectory
%             traj_y;
%             traj_z];
% 
% vel_x = gradient(traj_x, dt); % X velocity trajectory
% vel_y = gradient(traj_y, dt); % Y velocity trajectory
% vel_z = gradient(traj_z, dt); % Z velocity trajectory
% traj_vel = [vel_x; % Combined velocity trajectory
%             vel_y;
%             vel_z];
% 
% accel_x = gradient(vel_x, dt); % X acceleration trajectory
% accel_y = gradient(vel_y, dt); % Y acceleration trajectory
% accel_z = gradient(vel_z, dt); % Z acceleration trajectory
% traj_accel = [accel_x; % Combined acceleration trajectory
%               accel_y;
%               accel_z];
% traj_accel(3,:) = traj_accel(3,:) + 9.81; % Add gravity to Z-component
% 
% psiDes = 0; % Constant yaw value
% 
% % Initial conditions - [x, dx, y, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi]
% state = [5; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];

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
V = zeros(2,N); 
V(1,:) = Vel_x;
V(2,:) = Vel_y;

% Calculate acceleration due to drag
Cd = 0.75; % Drag coefficient estimate
A = 0.1; % Area estimate (m^2)
p = 1.225; % Air density (kg/m^3)
S = (1/2) * p * A * Cd; % Placeholder for 0.5*p*A*Cd in drag equation
F_wind = S .* V.^2; % Force due to wind

% a_wind = F_wind ./ m; % Acceleration due to wind if wind noise is desired

a_wind = zeros(2,N); % No wind noise

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% GAINS %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%

Kpz = 25.01;
Kiz = 0.97;
Kdz = 9.03;

Kpx = 6.45;
Kix = 0.089;
Kdx = 0.00011;

Kpy = 7.34;
Kiy = 0.000059;
Kdy = 0.514;

Kpphi = 8.86;
Kiphi = 0.0000435;
Kdphi = 0.00042;

Kptheta = 8.43;
Kitheta = 0.00062;
Kdtheta = 0.00011;

Kppsi = 3.51;
Kipsi = 0.008;
Kdpsi = 0.0053;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% INITIALIZE VARIABLES %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

prevPhi = state(7); % Initial global roll value for derivative error tracking
prevTheta = state(9); % Initial global pitch value for derivative error tracking
prevPsi = state(11); % Initial global yaw value for derivative error tracking
P = 0; % Initial body frame roll rate
Q = 0; % Initial body frame pitch rate
R = 0; % Initial body frame yaw rate

% Initial control inputs
T = g*m; % Thrust
Tphi = 0; % Roll torque
Ttheta = 0; % Pitch torque
Tpsi = 0; % Yaw torque
U = [T; % Combined inputs
     Tphi;
     Ttheta;
     Tpsi];

% Store and initialize control input values over time
UHist = zeros(4, length(t));
UHist(:,1) = [T; Tphi; Ttheta; Tpsi];

% Store data over time and populate inital values from intial state
xHist = zeros(1,length(t));
yHist = zeros(1,length(t));
zHist = zeros(1,length(t));
phiHist = zeros(1,length(t));
thetaHist = zeros(1,length(t));
psiHist = zeros(1,length(t));

xHist(1) = state(1);
yHist(1) = state(3);
zHist(1) = state(5);
phiHist(1) = state(7);
thetaHist(1) = state(9);
psiHist(1) = state(11);

% Store positional errors over time for performance analysis
e_x = zeros(1,N);
e_y = zeros(1,N);
e_z = zeros(1,N);

% Intialize integral error trackers
e_IZ = 0;
e_IX = 0;
e_IY = 0;
e_IPhi = 0;
e_ITheta = 0;
e_IPsi = 0;

% Integral Windup Limits
e_IZ_lim = 1.0;
e_IX_lim = 1.0;
e_IY_lim = 1.0;
e_IPhi_lim = 0.5;
e_ITheta_lim = 0.5;
e_IPsi_lim = 0.5;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% SIMULATION %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i = 1:N

    %%%% Z %%%%
    e_posZ = traj_pos(3,i) - state(5); % Positional error
    e_velZ = traj_vel(3,i) - state(6); % Velocity error
    e_IZ = e_IZ + e_posZ * dt; % Integral error component
    e_IZ = max(min(e_IZ, e_IZ_lim), -e_IZ_lim); % Limit integral error windup
    e_z(i) = e_posZ; % store error for RMSE calculation
    
    % Thrust control input (N)
    T = (m / (cos(state(7)) * cos(state(9)))) * (g + Kpz*e_posZ + Kiz*e_IZ + Kdz*e_velZ);
    T = min(max(T, 0), 25); % Limit thrust to operational range

    %%%% X %%%%
    e_posX = traj_pos(1,i) - state(1); % Positional error
    e_velX = traj_vel(1,i) - state(2); % Velocity error
    e_IX = e_IX + e_posX * dt; % Integral error component
    e_IX = max(min(e_IX, e_IX_lim), -e_IX_lim); % Limit integral error windup
    e_x(i) = e_posX; % store error for RMSE calculation

    % Desired pitch value given x-positional/velocity error
    thetaDes = (1/T) * (m*traj_accel(1,i) + Kpx*e_posX + Kix*e_IX + Kdx*e_velX);

    %%%% Y %%%%
    e_posY = traj_pos(2,i) - state(3); % Positional error
    e_velY = traj_vel(2,i) - state(4); % Velocity error
    e_IY = e_IY + e_posY * dt; % Integral error component
    e_IY = max(min(e_IY, e_IY_lim), -e_IY_lim); % Limit integral error windup
    e_y(i) = e_posY; % store error for RMSE calculation

    % Desired roll value given y-positional/velocity error
    phiDes = -(1/T) * (m*traj_accel(2,i) + Kpy*e_posY + Kiy*e_IY + Kdy*e_velY);
    
    %%%% Pitch Torque %%%%
    e_posTheta = thetaDes - state(9); % Angle error
    e_velTheta = (state(9) - prevTheta)/dt; % Angular velocity error
    e_ITheta = e_ITheta + e_posTheta * dt; % Integral error component
    e_ITheta = max(min(e_ITheta, e_ITheta_lim), -e_ITheta_lim); % Limit integral error windup
    prevTheta = state(9); % Reset previous angle value for derivative error tracking

    % Desired pitch torque given theta errors
    Ttheta = Kptheta*e_posTheta + Kitheta*e_ITheta + Kdtheta*e_velTheta;

    %%%% Roll Torque %%%%
    e_posPhi = phiDes - state(7); % Angle error
    e_velPhi = (state(7) - prevPhi)/dt; % Angular velocity error
    e_IPhi = e_IPhi + e_posPhi * dt; % Integral error component
    e_IPhi = max(min(e_IPhi, e_IPhi_lim), -e_IPhi_lim); % Limit integral error windup
    prevPhi = state(7); % Reset previous angle value for derivative error tracking

    % Desired roll torque given phi errors
    Tphi = Kpphi*e_posPhi + Kiphi*e_IPhi + Kdphi*e_velPhi;

    %%%% Yaw Torque %%%%
    e_posPsi = psiDes - state(11); % Angle error
    e_velPsi = (state(11) - prevPsi)/dt; % Angular velocity error
    e_IPsi = e_IPsi + e_posPsi * dt; % Integral error component
    e_IPsi = max(min(e_IPsi, e_IPsi_lim), -e_IPsi_lim); % Limit integral error windup
    prevPsi = state(11); % Reset previous angle value for derivative error tracking

    % Desired yaw torque given psi errors
    Tpsi = Kppsi*e_posPsi + Kipsi*e_IPsi + Kdpsi*e_velPsi;

    %%%% State Changes %%%%

    % Translation acceleration calculations with added wind disturbance
    ax = (T/m) * (sin(state(7)) * sin(state(11)) + cos(state(7)) * cos(state(11)) * sin(state(9))) + a_wind(1,i);
    ay = (T/m) * (cos(state(7)) * sin(state(9)) * sin(state(11)) - sin(state(7)) * cos(state(11))) + a_wind(2,i);
    az = -g + (T/m) * cos(state(7)) * cos(state(9));

    % Double integral to get new velocity and position due to updated
    % acceleration and time step dt
    state(2) = state(2) + ax*dt; % dx
    state(4) = state(4) + ay*dt; % dy
    state(6) = state(6) + az*dt; % dz

    state(1) = state(1) + state(2)*dt; % x
    state(3) = state(3) + state(4)*dt; % y
    state(5) = state(5) + state(6)*dt; % z

    % Body frame angular acceleration calculations
    dP = ((Iyy - Izz)/Ixx) * Q * R + (Tphi/Ixx); % roll accleration
    dQ = ((Izz - Ixx)/Iyy) * P * R + (Ttheta/Iyy); % pitch acceleration
    dR = ((Ixx - Iyy)/Izz) * P * Q + (Tpsi/Izz); % yaw acceleration
    
    % Integral to get body frame angular velocities due to new acceleration
    P = P + dP*dt; % roll velocity
    Q = Q + dQ*dt; % pitch velocity 
    R = R + dR*dt; % yaw velocity

    % Convert body frame angular velocities to global frame
    dPhi = P + R * cos(state(7)) * tan(state(9)) + Q * sin(state(7)) * tan(state(9)); % dRoll
    dTheta = Q * cos(state(7)) - R * sin(state(7)); % dPitch
    dPsi = (Q * sin(state(7)) + R * cos(state(7))) / cos(state(9)); % dYaw

    % Integral to get new angles based on angular velocities
    state(7) = state(7) + dPhi*dt; % Roll
    state(9) = state(9) + dTheta*dt; % Pitch
    state(11) = state(11) + dPsi*dt; % Yaw

    % Store all values
    xHist(i) = state(1);
    yHist(i) = state(3);
    zHist(i) = state(5);
    phiHist(i) = state(7);
    thetaHist(i) = state(9);
    psiHist(i) = state(11);

    UHist(:,i) = [T; Tphi; Ttheta; Tpsi];
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

fprintf('\nRoot Mean Squared Error for PID Trajectory Tracker\n')
fprintf('- - - - - - - - - - - - - - -\n');
fprintf('3-Dimensional RMSE: %.3f meters\n', rmse_3D)
fprintf('X-Position RMSE: %.3f meters\n', rmse_x)
fprintf('Y-Position RMSE: %.3f meters\n', rmse_y)
fprintf('Z-Position RMSE: %.3f meters\n', rmse_z)

% Calculate oscillations as a local minima/maxima
% Calculate 1-D velocities
dx = gradient(xHist, t);
dy = gradient(yHist, t);
dz = gradient(zHist, t);

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

% Position and angle states over time
state = [xHist;
        yHist;
        zHist;
        phiHist;
        thetaHist;
        psiHist];

% Reference trajectory over time
x_ref = [traj_pos;
        zeros(3,N)];

% Plot colors
colors = [0.8500 0.3250 0.0980; % 'o'
        0.4660 0.6740 0.1880; % 'g'
        0.3010 0.7450 0.9330; % 'c'
        0.4940 0.1840 0.5560]; % 'm'

% Plot states
figure;
labels = {'x (m)', 'y (m)', 'z (m)', '\phi (rad)', '\theta (rad)', '\psi (rad)'};
for j = 1:6
    subplot(3,2,j);
    plot(t, state(j,:), 'b', 'LineWidth', 1.3);
    hold on;
    plot(t, x_ref(j, :), 'k--', 'LineWidth', 1.1);
    ylabel(labels{j});
    xlabel('Time (s)');
    legend('Actual','Reference');
    grid on;
end
sgtitle('Quadcopter States with PID Controller');

% Plot inputs
figure;
labels = {'U1 - Thrust (N)', 'U2 - Roll Torque (Nm)', 'U3 - Pitch Torque (Nm)', 'U4 - Yaw Torque (Nm)'};
for k = 1:4
    subplot(2,2,k);
    plot(t, UHist(k,:), 'Color', colors(k,:), 'LineWidth', 1.4);
    xlabel('Time (s)');
    ylabel(labels{k});
    grid on;
end
sgtitle('Quadcopter Control Inputs with PID Controller')

% Plot 3D trajectory
figure;
plot3(traj_pos(1,:), traj_pos(2,:), traj_pos(3,:), 'k--', 'LineWidth', 1.5);
hold on;
plot3(xHist, yHist, zHist, 'b', 'LineWidth', 2);
hold on;
scatter3(traj_pos(1,1), traj_pos(2,1), traj_pos(3,1), 75, 'go', 'filled');
hold on;
scatter3(traj_pos(1,N), traj_pos(2,N), traj_pos(3,N), 75, 'ro', 'filled');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
legend('Reference Trajectory','Actual Trajectory','Start','End');
grid on;
axis equal;
title('3D Position for Trajectory Tracking with PID Controller');