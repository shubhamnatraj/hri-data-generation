function u_h_t = human_action_selection(x_h, goal_h, x_r_hat)
    
    params;

    u = sdpvar(2,1);
    
    x_h_t = human_dynamics(x_h, u, delta_T);
    objective = eta_1 * goal_objective_human(x_h_t, goal_h, u) + B * eta_2 * safety_objective_human(x_h_t, x_r_hat);

    constraint = [norm(u) <= u_h_max];

    ops = sdpsettings('verbose',0);

    optimize(constraint, objective, ops);

    u_h_t = value(u);

    
end

function x_next = human_dynamics(x, u, delta_T)
    params;
    x_next = x +  u * delta_T; 
end