% =========================================================================
% Main simulation script for Ackermann steering control
% =========================================================================

clear; clc; close all;

params = params();
fis = readfis('ackermann_fuzzy.fis');

%% ========================================================================
% 1) PID CONTROLLER
% =========================================================================
[state_pid, controller_pid] = init_state();
traj_pid = [];

rho_hist_pid = [];
t_pid = 0;

figure(1);
set(gcf, 'Name', 'PID', 'NumberTitle', 'off');

for t = 0:params.dt:params.T
    [xd, yd, rho, e_theta] = compute_error(state_pid, params);

    if rho < params.threshold_wp
        state_pid.current_wp = state_pid.current_wp + 1;

        controller_pid.rho_int = 0;
        controller_pid.etheta_int = 0;
        controller_pid.rho_prev = 0;
        controller_pid.etheta_prev = 0;

        if state_pid.current_wp > size(params.waypoints, 1)
            break;
        end
        continue;
    end

    % Store error
    rho_hist_pid = [rho_hist_pid; rho];
    t_pid = t;

    % PID Control
    [v, delta, controller_pid] = control_pid(rho, e_theta, controller_pid, params);

    % Update state
    state_pid = kinematics(state_pid, v, delta, params);
    traj_pid = [traj_pid; state_pid.x state_pid.y];

    plot_traj(state_pid, traj_pid, xd, yd, params, delta);
end

%% ========================================================================
% 2) FUZZY CONTROLLER
% =========================================================================
[state_fuzzy, ~] = init_state();
traj_fuzzy = [];

rho_hist_fuzzy = [];
t_fuzzy = 0;

figure(2);
set(gcf, 'Name', 'Fuzzy', 'NumberTitle', 'off');

for t = 0:params.dt:params.T
    [xd, yd, rho, e_theta] = compute_error(state_fuzzy, params);

    if rho < params.threshold_wp
        state_fuzzy.current_wp = state_fuzzy.current_wp + 1;

        if state_fuzzy.current_wp > size(params.waypoints, 1)
            break;
        end
        continue;
    end

    % Store error
    rho_hist_fuzzy = [rho_hist_fuzzy; rho];
    t_fuzzy = t;

    % Fuzzy Control
    [v, delta] = ackermann_control_fuzzy(rho, e_theta, fis);

    % Update state
    state_fuzzy = kinematics(state_fuzzy, v, delta, params);
    traj_fuzzy = [traj_fuzzy; state_fuzzy.x state_fuzzy.y];

    plot_traj(state_fuzzy, traj_fuzzy, xd, yd, params, delta);
end

%% ========================================================================
% 3) FINAL COMPARISON
% =========================================================================
figure(3);
set(gcf, 'Name', 'Comparison', 'NumberTitle', 'off');
hold on; grid on; axis equal;

plot(traj_pid(:,1), traj_pid(:,2), 'b', 'LineWidth', 2);
plot(traj_fuzzy(:,1), traj_fuzzy(:,2), 'm', 'LineWidth', 2);
plot(params.waypoints(:,1), params.waypoints(:,2), 'ro');

title('PID vs Fuzzy');
legend('PID', 'Fuzzy', 'Waypoints');
xlabel('X (m)');
ylabel('Y (m)');

%% ========================================================================
% 4) PERFORMANCE METRICS
% =========================================================================

% Time
time_pid = t_pid;
time_fuzzy = t_fuzzy;

% Average error
avg_error_pid = mean(rho_hist_pid);
avg_error_fuzzy = mean(rho_hist_fuzzy);

% Maximum error
max_error_pid = max(rho_hist_pid);
max_error_fuzzy = max(rho_hist_fuzzy);

% Trajectory length
dist_pid = sum(sqrt(sum(diff(traj_pid).^2, 2)));
dist_fuzzy = sum(sqrt(sum(diff(traj_fuzzy).^2, 2)));

%% FINAL PRINT
fprintf('\n================ COMPARISON =================\n');

fprintf('\n--- PID ---\n');
fprintf('Time: %.2f s\n', time_pid);
fprintf('Average error: %.4f\n', avg_error_pid);
fprintf('Maximum error: %.4f\n', max_error_pid);
fprintf('Distance traveled: %.2f m\n', dist_pid);

fprintf('\n--- FUZZY ---\n');
fprintf('Time: %.2f s\n', time_fuzzy);
fprintf('Average error: %.4f\n', avg_error_fuzzy);
fprintf('Maximum error: %.4f\n', max_error_fuzzy);
fprintf('Distance traveled: %.2f m\n', dist_fuzzy);

fprintf('\n=============================================\n');