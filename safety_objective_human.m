function y = safety_objective_human(x_h, x_r)
    params;

    y = theta_5 * exp(-1 * theta_6 * norm(x_h - x_r)^2 );
end