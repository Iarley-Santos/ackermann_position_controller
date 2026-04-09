% =========================================================================
% Parameters function
% =========================================================================

function params = params()
    % Simulation parameters
    params.dt = 0.05;      % Time step (50 ms)
    params.T = 100;        % Total simulation time (s)

    % Robot parameters
    params.L = 0.35;        % Wheelbase (m)
    params.W = 0.3;         % Robot width (m)
    params.Lf = params.L/2; % Distance from center to front axle (m)
    params.Lr = params.L/2; % Distance from center to rear axle (m)
    params.R_min = 1.3;     % Minimum curvature radius (m)
    params.delta_max = atan(params.L / params.R_min);
    params.v_max = 36.0;

    % Control parameters
    params.kp_vel = 3.0;   % Proportional gain for velocity
    params.ki_vel = 0.001; % Integral gain for velocity
    params.kd_vel = 0.2;   % Derivative gain for velocity

    params.kp_ang = 3.0;   % Proportional gain for angle
    params.ki_ang = 0.001; % Integral gain for angle
    params.kd_ang = 0.2;   % Derivative gain for angle

    params.threshold_wp = 0.5;  % Distance to switch waypoint (m)

    % Waypoints
    params.waypoints = [1  0;
                        5  5;
                        10 10;
                        0  0];

    % Robot geometry (for visualization)
    params.car_body = [params.Lf   params.W/2;
                       params.Lf  -params.W/2;
                      -params.Lr  -params.W/2;
                      -params.Lr   params.W/2;
                       params.Lf   params.W/2]';

    wheel_length = 0.2;
    wheel_width = 0.05;
    params.wheel_shape = [wheel_length/2   wheel_width/2;
                          wheel_length/2  -wheel_width/2;
                         -wheel_length/2  -wheel_width/2;
                         -wheel_length/2   wheel_width/2;
                          wheel_length/2   wheel_width/2]';

    params.wheels_positions = [params.Lf   params.W/2;    % Front right wheel
                                params.Lf  -params.W/2;   % Front left wheel
                               -params.Lr   params.W/2;   % Rear right wheel
                               -params.Lr  -params.W/2]'; % Rear left wheel
end