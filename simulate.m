clear;
params;

%% Logging

logsize = T_sim  / delta_T + 1 ;
log_quad_state_real = zeros(9, logsize, 2); % quad real state % Axis 3 - 0 is human, 1 is robot
log_quad_goal       = zeros(4, logsize, 2); % quad goal % Axis 3 - 0 is human, 1 is robot

% log_quad_state_real(:, iter_loop, iQuad) = [Quadrotor(iQuad).pos_real_; ...
%                                                     Quadrotor(iQuad).vel_real_; ...
%                                                     Quadrotor(iQuad).euler_real_];
%% Robot 

rng("shuffle");
x_r_t = generate_random_position_2d(grid_size); % State of Robot at time t = 0
goal_r = generate_random_position_2d(grid_size); % Goal position of Robot

u_r_t = [0 ; 0];

%% Human

% Moves along X -axis

x_h_t = generate_random_position_2d(grid_size); % State of Human at time t = 0
goal_h = generate_random_position_2d(grid_size); % Goal Position of Human

u_h_t = [0 ; 0];

% data format is ( t, x_h(1,2), u_h(1,2), g_h(1,2) , B, x_r(1,2), u_r(1,2), g_r(1,2)) (14 columns)
% (T_sim - 0 + 1) / delta_T entries (rows) in the trajectory


% nRows = T_sim  / delta_T + 1 ;
% nCols = length([t, x_h_t(1), x_h_t(2), u_h_t(1), u_h_t(2), goal_h(1), goal_h(2), B, x_r_t(1), x_r_t(2), u_r_t(1), u_r_t(2), goal_r(1), goal_r(2)]);
% 
% trajData = zeros(nRows,nCols);
idx = 1;

%%
% x_h_log_x = [];
% x_h_log_y = [];
% x_r_log_x = [];
% x_r_log_y = [];
% Main Simulation Loop
for t=0:delta_T:T_sim
    
    disp(t)
    % trajData(idx,:) = [t, x_h_t(1), x_h_t(2), u_h_t(1), u_h_t(2), goal_h(1), goal_h(2), B, x_r_t(1), x_r_t(2), u_r_t(1), u_r_t(2), goal_r(1), goal_r(2)];
    % idx = idx + 1;

    % Human - Index 0
    log_quad_state_real(:, idx, 1) = [x_h_t ; B; ...
                                      u_h_t ; 0; ...
                                      zeros(3,1)];
    log_quad_goal(:, idx, 1) = [goal_h ; 0 ; 0];
    
    % Robot - Index 1
    log_quad_state_real(:, idx, 2) = [x_r_t ; 0; ...
                                      u_r_t ; 0; ...
                                      zeros(3,1)];
    log_quad_goal(:, idx, 2) = [goal_r ; 0 ; 0];
    
    idx = idx + 1 ;

        if t == 250
            B = 0;
        end
    % If either Human or Robot has reached its goal, update
        if norm(x_h_t - goal_h) < goal_reach_dist
            x_h_t = generate_random_position_2d(grid_size);
            disp("Updated Human Goal Position")
        end

        if norm(x_r_t - goal_r) < goal_reach_dist
            x_r_t = generate_random_position_2d(grid_size);
            disp("Updated Robot Goal Position")
        end

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

end

date_now = datestr(now, 'yyyymmdd_HHMMSS');
current_dir = pwd;
save_dir = [current_dir, '/logs/'];
file_name = strcat(save_dir,'log_', date_now,'.mat');
save(file_name, '-regexp', '^log');
fprintf('[%s] Saved log data to %s \n', datestr(now, 'HH:MM:SS'), file_name);

% plot(x_h_log_x, x_h_log_y, "--.r")
% hold on
% plot(x_r_log_x, x_r_log_y, "--.b")
% title("Sample trajectory with \beta = 1");
% legend('Human Position', 'Robot Position')


% if logTrajectories == 1
%     filename = strcat(dataFolder,['trajectory_' num2str(fNumber) '.csv']);
%     writematrix(trajData, filename)
% end

function pos = generate_random_position_2d(grid_size)
    pos = grid_size * rand(2,1);
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