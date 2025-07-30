clc; clear; close all; 
format long;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% Genetic Algorithm Gain Optimization for LQI Controller %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% Trajectories %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Uncomment trajectory of choice 

% Quadcopter parameters
m = 1.25; % Mass (kg)
g = 9.81; % Gravity (m/s^2)
Ixx = 0.0232; % Inertia (kg m^2)
Iyy = 0.0232; % Inertia (kg m^2)
Izz = 0.0468; % Inertia (kg m^2)

%%%% 3D LINE %%%%
% dt = 0.01; % Time step (s)
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

vel_x = -r * w * sin(w*t); % X velocity trajectory
vel_y = r * w * cos(w*t); % Y velocity trajectory
vel_z = linearRate * ones(1,N); % Z velocity trajectory

% Generate 12 state reference trajectory
x_ref = zeros(12, N); % [x; dx; y; dy; z; dz; phi; dphi; theta; dtheta; psi; dpsi]
x_ref(1, :) = traj_x;
x_ref(2, :) = vel_x;
x_ref(3, :) = traj_y;
x_ref(4, :) = vel_y;
x_ref(5, :) = traj_z;
x_ref(6, :) = vel_z;

%%%% Reference Inputs %%%%
% Assume desired state near hover at each time step for slower, controlled
% trajectories
U_ref = [m*g; 0; 0; 0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% INITIAL Q & R COSTS %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% QR = [Q, R]
QR = [500, 230, 360, 224, 500, 460, 186, 7.7, 84.8, 16.5, 118, 295, 384, 175, 0.127, 0.122, 0.518, 2.29, 9.96];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% STATE SPACE MODEL %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Number of states tracked with integral action
n_int = 3; % (x, y, z)

A = zeros(12); % Initialize A-matrix
B = zeros(12, 4); % Initialize B-matrix
C = zeros(n_int, 12); % Initialize C-matrix
D = zeros(n_int, 4); % Initialize D-matrix

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

% Populate C-matrix
C(1,1) = -1;
C(2,3) = -1;
C(3,5) = -1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% GENETIC ALGORITHM %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This uses the controller packaged as a weight RMSE cost function to run
% an iterative genetic algorithm optimziation problem, outputting the
% optimized cost array

function RMSE_cost = LQI(A, B, C, D, QR, U_ref, x_ref)
    
    % Trajectory and platform parameters
    dt = 0.01; % Time step (s)
    % tf = 10; % Final time (s) for 3D line
    tf = 30; % Final time (s) for spiral
    t = 0:dt:tf; % Time array (s)
    N = length(t); % Number of time steps
    m = 1.25; % Mass (kg)
    g = 9.81; % Gravity (m/s^2)
    
    % Number of states tracked with integral action
    n_int = 3; % (x, y, z, psi)
    
    % % 3D Line Initial conditions - [x, dx, y, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi]
    % state = zeros(12,N); % Tracks state over time
    % state(:,1) = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];

    % Spiral Initial conditions - [x, dx, y, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi]
    state = zeros(12,N); % Tracks state over time
    state(:,1) = [5; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    
    % Error Tracking
    e_x = zeros(1,N);
    e_y = zeros(1,N);
    e_z = zeros(1,N);
    e_phi = zeros(1,N);
    e_theta = zeros(1,N);
    e_psi = zeros(1,N);
    
    % Extract costs into Q and R arrays
    Q_mod = diag(QR(1:15));
    R = diag(QR(16:19));
    
    % Create state space form from state matrices
    sys = ss(A, B, C, D);
    
    % Compute Gains for states and integral action states
    [K,S,P] = lqi(sys, Q_mod, R);
    
    Kx = K(:, 1:12); % State gains
    Ki = K(:, 13:end); % Integral state gains
    
    % Initialize variables
    e_int = zeros(n_int, 1); % Tracks integral error between iterations
    
    % Run controller
    for i = 1:N-1
    
        % State error
        e = state(:,i) - x_ref(:,i);
        
        % Update integral error
        e_int = e_int + e([1,3,5])*dt;
    
        % Calculate system control input from state error
        U = U_ref - Kx * e - Ki * e_int;
    
        % Limit inputs to operational ranges
        U(1) = min(max(U(1), -m*g), 25 - m*g); % limit between free fall (0 N thrust) and +25 N
        U(2) = min(max(U(2), -7), 7);
        U(3) = min(max(U(3), -7), 7);
        U(4) = min(max(U(4), -7), 7);
    
        % Track positional error over time - store up for RMSE calculation
        e_x(i) = e(1);
        e_y(i) = e(3);
        e_z(i) = e(5);
        e_phi(i) = e(7);
        e_theta(i) = e(9);
        e_psi(i) = e(11);
    
        % Calculate state change at time step
        dx = A * state(:,i) + B * U;
        dx(6) = dx(6) - g; % Account for gravity not represented in linearized state space form
    
        % Calculate updated state
        state(:,i+1) = state(:,i) + dx * dt;
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
vars = 19;

% Upper and lower bounds on terms - estimated and tuned over time 
% (x, y, z, phi, theta, psi)
lb = [zeros(1,15), 0.001*ones(1,4)];  % Q ≥ 0, R > 0
ub = [500*ones(1,15), 10*ones(1,4)];

% % GA setup for starting with random costs
% opts = optimoptions('ga', ...
%                     'InitialPopulationMatrix', rand(38, vars) .* (ub - lb) + lb, ...
%                     'MaxGenerations',1000, ... % Iterations
%                     'PopulationSize', 38, ... % Cost population size (2*number of vars)
%                     'Display','iter');

% GA setup for follow-on runs with predefined costs as a seed
randVars = rand(37, vars) .* (ub - lb) + lb; % Random costs
seedVars = [QR; randVars]; % Seed of chosen costs plus random costs

opts = optimoptions('ga', ...
                    'InitialPopulationMatrix', seedVars, ...
                    'MaxGenerations',500, ... % Iterations
                    'PopulationSize', 38, ... % Cost population size (2*number of vars)
                    'Display','iter');

% Define cost/fitness function
cost = @(QR) LQI(A, B, C, D, QR, U_ref, x_ref);

% Run genetic algorithm with cost/fitness function, bounds, and intial cost
% population
[bestGains, bestCost] = ga(cost, vars, [], [], [], [], lb, ub, [], opts);

% Display best weights to terminal after running
bestGains