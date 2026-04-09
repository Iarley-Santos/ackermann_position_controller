% =========================================================================
% Initialize state function
% =========================================================================

function [state, controller] = init_state()
    % Initial robot state
    state.x = 0;
    state.y = 0;
    state.theta = 0;
    state.current_wp = 1;

    % Controller state (integral and derivative)
    controller.rho_int = 0;
    controller.rho_prev = 0;
    controller.etheta_int = 0;
    controller.etheta_prev = 0;
end