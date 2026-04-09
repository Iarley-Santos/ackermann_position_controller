% =========================================================================
% Plot trajectory function
% =========================================================================

function plot_traj(state, traj, xd, yd, params, delta)
    clf;
    hold on;
    grid on;
    axis equal;

    % Auto adjust plot limits
    margin = 1;
    if size(traj, 1) > 1
        all_x = [traj(:, 1); xd];
        all_y = [traj(:, 2); yd];
        xmin = min(all_x) - margin;
        xmax = max(all_x) + margin;
        ymin = min(all_y) - margin;
        ymax = max(all_y) + margin;
        xlim([xmin xmax]);
        ylim([ymin ymax]);
    end

    % Plot traveled trajectory
    plot(traj(:, 1), traj(:, 2), 'b', 'LineWidth', 2);

    % Plot current waypoint
    plot(xd, yd, 'ro', 'MarkerSize', 10, 'LineWidth', 3);

    % Plot all waypoints
    plot(params.waypoints(:, 1), params.waypoints(:, 2), 'r.', 'MarkerSize', 15);

    % Rotation matrix
    R = [cos(state.theta) -sin(state.theta);
         sin(state.theta)  cos(state.theta)];

    % Plot robot body
    car_global = R * params.car_body + [state.x; state.y];
    plot(car_global(1, :), car_global(2, :), 'k', 'LineWidth', 2);

    % Plot wheels
    for i = 1:4
        wx = params.wheels_positions(1, i);
        wy = params.wheels_positions(2, i);

        % Front wheels have steering angle
        if i <= 2
            Rw = [cos(state.theta + delta) -sin(state.theta + delta);
                  sin(state.theta + delta)  cos(state.theta + delta)];
        else
            Rw = R;
        end

        wheel_global = Rw * params.wheel_shape + R * [wx; wy] + [state.x; state.y];
        plot(wheel_global(1, :), wheel_global(2, :), 'r', 'LineWidth', 2);
    end

    % Plot robot direction
    quiver(state.x, state.y, cos(state.theta), sin(state.theta), 0.5, 'k', 'LineWidth', 2);

    % Information in title
    title(sprintf('Ackermann | WP: %d/%d', ...
         state.current_wp, size(params.waypoints, 1)));
    xlabel('X (m)');
    ylabel('Y (m)');

    drawnow;
end