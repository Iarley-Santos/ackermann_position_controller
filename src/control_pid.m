% =========================================================================
% PID control function
% =========================================================================

function [v, delta, controller] = control_pid(rho, e_theta, controller, params)
    % Update integrators and derivatives
    controller.rho_int = controller.rho_int + rho * params.dt;
    rho_der = (rho - controller.rho_prev) / params.dt;
    controller.rho_prev = rho;

    controller.etheta_int = controller.etheta_int + e_theta * params.dt;
    etheta_der = (e_theta - controller.etheta_prev) / params.dt;
    controller.etheta_prev = e_theta;

    % PID control law
    v = params.kp_vel * rho + params.ki_vel * controller.rho_int + params.kd_vel * rho_der;
    omega = params.kp_ang * e_theta + params.ki_ang * controller.etheta_int + params.kd_ang * etheta_der;

    % Conversion to Ackermann steering angle
    if abs(v) < 0.01
        delta = 0;
    else
        delta = atan(params.L * omega / v);
    end
    
    % Velocity saturation
    v = max(min(v, params.v_max), 0);
   
    % Steering angle saturation
    delta = max(min(delta, params.delta_max), -params.delta_max);
end