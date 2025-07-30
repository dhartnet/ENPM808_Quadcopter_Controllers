clc; clear; close all; 
format long;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% Genetic Algorithm Gain Optimization for PID Controller %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% Trajectories %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Uncomment trajectory of choice

%%%% 3D LINE %%%%
% dt = 0.01; % Time step (s)
% tf = 10; % Final time (s)
% t = 0:dt:tf; % Time array (s)
% N = length(t); % Number of time steps
% 
% x_0 = [0; 0; 0]; % Starting point (x,y,z)
% x_f = [-10; 10; 10]; % Final point (x,y,z)
% 
% traj_pos = x_0 + (x_f - x_0) * (t / tf); % Positional trajectory
% traj_vel = repmat((x_f - x_0) / tf, 1, N); % Derivative for velocity
% traj_accel = [0; 0; 0] * t; % Acceleration trajectory
% traj_accel(3,:) = traj_accel(3,:) + 9.81; % Add gravity to z-trajectory

%%%% UPWARDS SPIRAL %%%%
dt = 0.01; % Time step (s)
tf = 30; % Final time (s)
t = 0:dt:tf; % Time array (s)
N = length(t); % Number of time steps

r = 5; % Radius of spiral (m)
w = 2 * pi / (tf/3); % Angular velocity (rad/s)
linearRate = 0.75; % Linear rate of climb (m/s)

traj_x = r * cos(w*t); % X position trajectory
traj_y = r * sin(w*t); % Y position trajectory
traj_z = linearRate * t; % Z position trajectory
traj_pos = [traj_x; % Combined (x,y,z) trajectory
            traj_y;
            traj_z];

vel_x = -r * w * sin(w*t); % X velocity trajectory
vel_y = r * w * cos(w*t); % Y velocity trajectory
vel_z = linearRate * ones(1,N); % Z velocity trajectory
traj_vel = [vel_x; % Combined velocity trajectory
            vel_y;
            vel_z];

accel_x = -r * w^2 * cos(w*t); % X acceleration trajectory
accel_y = -r * w^2 * sin(w*t); % Y acceleration trajectory
accel_z = 0 * t; % Z acceleration trajectory
traj_accel = [accel_x; % Combined acceleration trajectory
              accel_y;
              accel_z];
traj_accel(3,:) = traj_accel(3,:) + 9.81; % Add gravity to Z-component

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% INITIAL GAINS %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Kpz = 25;
Kiz = 0.65;
Kdz = 9.08;

Kpx = 6.42;
Kix = 0.087;
Kdx = 0.00023;

Kpy = 7.34;
Kiy = 0.00011;
Kdy = 0.5;

Kpphi = 8.85;
Kiphi = 0.00015;
Kdphi = 0.00042;

Kptheta = 8.42;
Kitheta = 0.0077;
Kdtheta = 0.00022;

Kppsi = 3.51;
Kipsi = 0.019;
Kdpsi = 0.0056;

% Combine gains into one array that cost function can read
gains = [Kpz, Kiz, Kdz, Kpx, Kix, Kdx, Kpy, Kiy, Kdy, Kpphi, Kiphi, Kdphi, Kptheta, Kitheta, Kdtheta, Kppsi, Kipsi, Kdpsi];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% GENETIC ALGORITHM %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This uses the controller packaged as a weight RMSE cost function to run
% an iterative genetic algorithm optimziation problem, outputting the
% optimized gain array

function RMSE_cost = PID(gains, traj_pos, traj_vel, traj_accel)
    
    % Trajectory and platform parameters
    dt = 0.01; % Time step (s)
    % tf = 10; % Line final time (s)
    tf = 30; % Spiral final time (s)
    t = 0:dt:tf; % Time array (s)
    N = length(t); % Number of time steps
    m = 1.25; % Mass (kg)
    g = 9.81; % Gravity (m/s^2)
    Ixx = 0.0232; % Inertia (kg m^2)
    Iyy = 0.0232; % Inertia (kg m^2)
    Izz = 0.0468; % Inertia (kg m^2)
    
    % Constant yaw value for all trajectories
    psiDes = 0;
    
    % % 3D Line Initial conditions - [x, dx, y, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi]
    % state = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    
    % Spiral Initial conditions - [x, dx, y, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi]
    state = [5; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    
    % Initialize variables
    prevPhi = state(7); % Initial global roll value for derivative error tracking
    prevTheta = state(9); % Initial global pitch value for derivative error tracking
    prevPsi = state(11); % Initial global yaw value for derivative error tracking
    P = 0; % Initial body frame roll rate
    Q = 0; % Initial body frame pitch rate
    R = 0; % Initial body frame yaw rate
    
    % Initialize control inputs
    T = g*m; % Thrust
    Tphi = 0; % Roll torque
    Ttheta = 0; % Pitch torque
    Tpsi = 0; % Yaw torque
    U = [T; % Combined inputs
         Tphi;
         Ttheta;
         Tpsi];
    
    % Store positional errors over time for cost analysis
    e_x = zeros(1,N);
    e_y = zeros(1,N);
    e_z = zeros(1,N);
    e_phi = zeros(1,N);
    e_theta = zeros(1,N);
    e_psi = zeros(1,N);
    
    % Split gains into variables
    gains = num2cell(gains);
    [Kpz,Kiz,Kdz,Kpx,Kix,Kdx,Kpy,Kiy,Kdy,Kpphi,Kiphi,Kdphi,Kptheta,Kitheta,Kdtheta,Kppsi,Kipsi,Kdpsi] = deal(gains{:});
    
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
    
    % Run controller
    for i = 1:N
    
        %%%% Z %%%%
        e_posZ = traj_pos(3,i) - state(5); % Positional error
        e_velZ = traj_vel(3,i) - state(6); % Velocity error
        e_IZ = e_IZ + e_posZ * dt; % Integral error component
        e_IZ = max(min(e_IZ, e_IZ_lim), -e_IZ_lim); % Limit integral error windup
        e_z(i) = e_posZ; % store error for RMSE calculation
        if isnan(e_z(i)) % If error is NaN, assign large value to ensure GA works
                e_z(i) = 30;        
        end
        
        % Thrust control input (N)
        T = (m / (cos(state(7)) * cos(state(9)))) * (g + Kpz*e_posZ + Kiz*e_IZ + Kdz*e_velZ);
        T = min(max(T, 0), 25); % Limit thrust to operational range
    
        %%%% X %%%%
        e_posX = traj_pos(1,i) - state(1); % Positional error
        e_velX = traj_vel(1,i) - state(2); % Velocity error
        e_IX = e_IX + e_posX * dt; % Integral error component
        e_IX = max(min(e_IX, e_IX_lim), -e_IX_lim); % Limit integral error windup
        e_x(i) = e_posX; % store error for RMSE calculation
        if isnan(e_x(i)) % If error is NaN, assign large value to ensure GA works
                e_x(i) = 30;        
        end
    
        % Desired pitch value given x-positional/velocity error
        thetaDes = (1/T) * (m*traj_accel(1,i) + Kpx*e_posX + Kix*e_IX + Kdx*e_velX);
    
        %%%% Y %%%%
        e_posY = traj_pos(2,i) - state(3); % Positional error
        e_velY = traj_vel(2,i) - state(4); % Velocity error
        e_IY = e_IY + e_posY * dt; % Integral error component
        e_IY = max(min(e_IY, e_IY_lim), -e_IY_lim); % Limit integral error windup
        e_y(i) = e_posY; % store error for RMSE calculation
        if isnan(e_y(i)) % If error is NaN, assign large value to ensure GA works
                e_y(i) = 30;        
        end
    
        % Desired roll value given y-positional/velocity error
        phiDes = -(1/T) * (m*traj_accel(2,i) + Kpy*e_posY + Kiy*e_IY + Kdy*e_velY);
        
        %%%% Pitch Torque %%%%
        e_posTheta = thetaDes - state(9); % Angle error
        e_velTheta = (state(9) - prevTheta)/dt; % Angular velocity error
        e_ITheta = e_ITheta + e_posTheta * dt; % Integral error component
        e_ITheta = max(min(e_ITheta, e_ITheta_lim), -e_ITheta_lim); % Limit integral error windup
        prevTheta = state(9); % Reset previous angle value for derivative error tracking
        if isnan(e_theta(i)) || abs(e_theta(i)) > 2*pi % If error is NaN, assign large value to ensure GA works
                e_theta(i) = 2*pi;        
        end
    
        % Desired pitch torque given theta errors
        Ttheta = Kptheta*e_posTheta + Kitheta*e_ITheta + Kdtheta*e_velTheta;
    
        %%%% Roll Torque %%%%
        e_posPhi = phiDes - state(7); % Angle error
        e_velPhi = (state(7) - prevPhi)/dt; % Angular velocity error
        e_IPhi = e_IPhi + e_posPhi * dt; % Integral error component
        e_IPhi = max(min(e_IPhi, e_IPhi_lim), -e_IPhi_lim); % Limit integral error windup
        prevPhi = state(7); % Reset previous angle value for derivative error tracking
        if isnan(e_phi(i)) || abs(e_phi(i)) > 2*pi % If error is NaN, assign large value to ensure GA works
                e_phi(i) = 2*pi;       
        end
    
        % Desired roll torque given phi errors
        Tphi = Kpphi*e_posPhi + Kiphi*e_IPhi + Kdphi*e_velPhi;
    
        %%%% Yaw Torque %%%%
        e_posPsi = psiDes - state(11); % Angle error
        e_velPsi = (state(11) - prevPsi)/dt; % Angular velocity error
        e_IPsi = e_IPsi + e_posPsi * dt; % Integral error component
        e_IPsi = max(min(e_IPsi, e_IPsi_lim), -e_IPsi_lim); % Limit integral error windup
        prevPsi = state(11); % Reset previous angle value for derivative error tracking
        if isnan(e_psi(i)) || abs(e_psi(i)) > 2*pi % If error is NaN, assign large value to ensure GA works
                e_psi(i) = 2*pi;        
        end
    
        % Desired yaw torque given psi errors
        Tpsi = Kppsi*e_posPsi + Kipsi*e_IPsi + Kdpsi*e_velPsi;
    
        %%%% State Changes %%%%
    
        % Translation acceleration calculations
        ax = (T/m) * (sin(state(7)) * sin(state(11)) + cos(state(7)) * cos(state(11)) * sin(state(9)));
        ay = (T/m) * (cos(state(7)) * sin(state(9)) * sin(state(11)) - sin(state(7)) * cos(state(11)));
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
    end
    
    % Weighted RMSE cost calculation
    w_x = 1.0;
    w_y = 1.0;
    w_z = 1.0;
    w_phi = 1.0;
    w_theta = 1.0;
    w_psi = 1.0;
    
    RMSE_cost = sqrt(mean(w_x * e_x.^2 + w_y * e_y.^2 + w_z * e_z.^2 + w_phi * e_phi.^2 + w_theta * e_theta.^2 + w_psi * e_psi.^2));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% SETUP AND RUN GA %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Number of variables to optimize
vars = 18;

% Upper and lower bounds on terms - estimated and tuned over time 
% (x, y, z, phi, theta, psi)
ub = [30 1 10,  20 0.5 2,  20 0.5 2,   10 0.3 0.3,   10 0.3 0.3,   10 0.3 0.3];
lb = [0 0 0,  0 0 0,  0 0 0,  0 0 0,   0 0 0,   0 0 0];

% GA setup for starting with random gains
opts = optimoptions('ga', ...
                    'InitialPopulationMatrix', rand(36, vars) .* (ub - lb) + lb, ...
                    'MaxGenerations',3000, ... % Iterations
                    'PopulationSize', 36, ... % Gain population size (2*number of vars)
                    'Display','iter');

% % GA setup for follow-on runs with predefined gains as a seed
% randVars = rand(35, vars) .* (ub - lb) + lb; % Random gains
% seedVars = [gains; randVars]; % Seed of chosen gains plus random gains
% 
% opts = optimoptions('ga', ...
%                     'InitialPopulationMatrix', seedVars, ...
%                     'MaxGenerations',3000, ... % Iterations
%                     'PopulationSize', 36, ... % Gain population size (2*number of vars)
%                     'Display','iter');

% Define cost/fitness function
cost = @(gains) PID(gains, traj_pos, traj_vel, traj_accel);

% Run genetic algorithm with cost/fitness function, bounds, and intial gain
% population
[bestGains, bestCost] = ga(cost, vars, [], [], [], [], lb, ub, [], opts);

% Display best gains to terminal after running
bestGains