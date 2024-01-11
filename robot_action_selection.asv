function u_r_t = robot_action_selection(x_r_t, goal_r)
    params;

    u = sdpvar(2,1);
    
    x_r_t = robot_dynamics(x_r_t, u, delta_T);
    
    objective = theta_1 * norm(x_r_t - goal_r)^2 + theta_2 * norm(u)^2;

    constraint = [norm(u) <= u_r_max];

    ops = sdpsettings('verbose',0);

    optimize(constraint, objective, ops);

    u_r_t = value(u);
end

function x_next = robot_dynamics(x, u, delta_T)
    params;
    x_next = x +  u * delta_T; 
end