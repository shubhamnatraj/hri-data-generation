function simulate(fNumber, dataFolder)


params;
close all
%% Robot 

rng("shuffle");
x_r_t = grid_size * rand(2,1); % State of Robot at time t = 0
goal_r = grid_size * rand(2,1); % Goal position of Robot

u_r_t = [0 ; 0];

%% Human

% Moves along X -axis

x_h_t = grid_size * rand(2,1); % State of Human at time t = 0
goal_h = grid_size * rand(2,1); % Goal Position of Human

u_h_t = [0 ; 0];

% data format is ( t, x_h(1,2), u_h(1,2), g_h(1,2) , B, x_r(1,2), u_r(1,2), g_r(1,2)) (14 columns)
% (T_sim - 0 + 1) / delta_T entries (rows) in the trajectory

t = 0;

nRows = (T_sim - 0 + 1) / delta_T;
nCols = length([t, x_h_t(1), x_h_t(2), u_h_t(1), u_h_t(2), goal_h(1), goal_h(2), B, x_r_t(1), x_r_t(2), u_r_t(1), u_r_t(2), goal_r(1), goal_r(2)]);

trajData = zeros(nRows,nCols);
idx = 1;

%%
% x_h_log_x = [];
% x_h_log_y = [];
% x_r_log_x = [];
% x_r_log_y = [];
% Main Simulation Loop
for t=0:delta_T:T_sim
    
    
    trajData(idx,:) = [t, x_h_t(1), x_h_t(2), u_h_t(1), u_h_t(2), goal_h(1), goal_h(2), B, x_r_t(1), x_r_t(2), u_r_t(1), u_r_t(2), goal_r(1), goal_r(2)];
    idx = idx + 1;
    % Find Human Action by solving optimization problem

        x_r_hat = observe_robot_state(x_r_t, std_dev);
        
        u_h_t = human_action_selection(x_h_t, goal_h, x_r_hat );

    % Find Robot Action
        u_r_t = robot_action_selection(x_r_t, goal_r);

    % Calculate Robot State from state and action at last time step
        x_r_t = robot_dynamics(x_r_t, u_r_t, delta_T);

    % Calculate Human State from state and action at last time step
        x_h_t = human_dynamics(x_h_t, u_h_t, delta_T);

    % Update the goal position if reached

    % x_h_log_x = [x_h_log_x x_h_t(1)];
    % x_h_log_y = [x_h_log_y x_h_t(2)];
    % x_r_log_x = [x_r_log_x x_r_t(1)];
    % x_r_log_y = [x_r_log_y x_r_t(2)];
    % 

end

% plot(x_h_log_x, x_h_log_y, "--.r")
% hold on
% plot(x_r_log_x, x_r_log_y, "--.b")
% title("Sample trajectory with \beta = 1");
% legend('Human Position', 'Robot Position')


if logTrajectories == 1
    filename = strcat(dataFolder,['trajectory_' num2str(fNumber) '.csv']);
    writematrix(trajData, filename)
end

end

function x_next = robot_dynamics(x, u, delta_T)
    x_next = x +  u * delta_T; 
end

function x_next = human_dynamics(x, u, delta_T)
    
    x_next = x +  u * delta_T; 
end

function x_r_hat = observe_robot_state(x_r_t, std_dev) 
    x_r_hat = x_r_t + std_dev * randn(2, 1) ;
end