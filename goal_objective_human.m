function y = goal_objective_human(x_h, goal_h, u_h)
    params; 
    y = theta_3 * norm(x_h - goal_h)^2 + theta_4 * norm(u_h)^2;
end
