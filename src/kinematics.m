% =========================================================================
% Kinematics function
% =========================================================================

function state = kinematics(state, v, delta, params)
    % Ackermann kinematic model
    state.theta = state.theta + (v / params.L) * tan(delta) * params.dt;
    state.x = state.x + v * cos(state.theta) * params.dt;
    state.y = state.y + v * sin(state.theta) * params.dt;
end