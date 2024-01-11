%% Robot Parameters

theta_1 = 1;
theta_2 = 0.5;

u_r_max = 2;

%% Human Parameters

eta_1 = 1;
eta_2 = 1;

theta_3 = 2.5;
theta_4 = 8 * 10^-3;
theta_5 = 300;
theta_6 = 6 * 10^-3;

B = 0;

u_h_max = 0.5;

std_dev = 1; % Gaussian Estimation error (Human Observing Robot's state)

%% Simulation Parameters

delta_T = 0.2;
T_sim = 15;

grid_size = 10; % square

numSimulations = 1; % no of trajectories
logTrajectories = 0 ; % 0 or 1