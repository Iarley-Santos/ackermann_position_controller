% =========================================================================
% Compute error function
% =========================================================================

function [xd, yd, rho, e_theta] = compute_error(state, params)
    xd = params.waypoints(state.current_wp, 1);
    yd = params.waypoints(state.current_wp, 2);

    ex = xd - state.x;
    ey = yd - state.y;

    rho = sqrt(ex^2 + ey^2);
    theta_d = atan2(ey, ex);
    e_theta = wrapToPi(theta_d - state.theta);
end