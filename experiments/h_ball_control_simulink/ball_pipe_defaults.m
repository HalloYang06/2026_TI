function bp = ball_pipe_defaults()
%BALL_PIPE_DEFAULTS Default parameters for the nonlinear steel-ball/PVC-pipe model.
%
% Geometry assumption:
%   A solid steel ball moves along the axis of a straight PVC pipe (or a
%   half-pipe rail). An actuator changes the pipe inclination angle.
%
% IMPORTANT:
%   The friction values below are engineering starting values, not material
%   constants. Replace them with values identified on the real apparatus.

bp.meta.model_name = 'ball_pipe_nonlinear';
bp.meta.version = '1.4-measured-linkage-115200-200hz-imu';

% Physical geometry and gravity.
bp.gravity = 9.80665;                 % m/s^2
bp.ball.radius = 0.005;               % m, approx. 10 mm measured diameter
bp.ball.density = 7850;               % kg/m^3, typical carbon/bearing steel
bp.ball.mass = (4/3) * pi * bp.ball.radius^3 * bp.ball.density;
bp.ball.inertia = (2/5) * bp.ball.mass * bp.ball.radius^2;
bp.pipe.usable_length = 0.224;        % m, physical ball path: +/-112 mm
bp.pipe.inner_diameter = 0.050;       % m, documented for later geometry upgrades

% Steel/PVC contact model.
% mu_s and mu_k act on BALL-PIPE SLIP velocity, not directly on rolling speed.
bp.contact.mu_static = 0.050;          % provisional low-friction measured regime
bp.contact.mu_kinetic = 0.030;         % replace with the user's exact result
bp.contact.stribeck_velocity = 0.020;  % m/s
bp.contact.slip_smoothing = 0.002;     % m/s; smaller is sharper but stiffer
bp.contact.slip_viscous = 0.010;       % N/(m/s)
bp.contact.rolling_resistance = 0.002; % low-resistance provisional value
bp.contact.omega_smoothing = 0.50;     % rad/s

% Other longitudinal resistance.
bp.drag.viscous = 0.012;               % N/(m/s)
bp.drag.quadratic = 0.0015;            % N/(m/s)^2

% RobStride RS00 official actuator data.
bp.rs00.rated_voltage = 48;             % V
bp.rs00.voltage_range = [24, 60];       % V
bp.rs00.gear_ratio = 10;
bp.rs00.rated_torque = 5.0;             % N*m
bp.rs00.peak_torque = 14.0;             % N*m
bp.rs00.no_load_speed = 315*pi/30;      % rad/s at output
bp.rs00.rated_speed = 100*pi/30;        % rad/s; conservative current spec
bp.rs00.torque_constant = 1.48;         % N*m/Arms
bp.rs00.rated_phase_current_peak = 4.7; % A peak
bp.rs00.maximum_phase_current_peak = 15.5; % A peak
bp.rs00.encoder_counts = 2^14;          % 14-bit single-turn absolute
bp.rs00.can_bitrate = 1e6;              % bit/s, CAN 2.0 extended frame

% Recommended RS00 motion-control-mode settings for first simulation.
% The outer LQI commands a PIPE ANGLE. The linkage inverse kinematics
% converts it to an RS00 output-shaft angle before the impedance loop.
bp.rs00.control_mode = 'motion';
bp.rs00.motion_kp = 40.0;               % N*m/rad, tune on the real mechanism
bp.rs00.motion_kd = 2.0;                % N*m/(rad/s), near-critical for default inertia
bp.rs00.feedforward_torque = 0.0;       % N*m
bp.rs00.user_torque_limit = 5.0;        % N*m; start at rated, not peak torque
bp.rs00.user_speed_limit = 9.0;         % rad/s; below default rated-speed estimate
bp.rs00.current_loop_time_constant = 0.0015; % s, identify with logged torque
bp.rs00.command_delay = 0.0045;         % s, 500 Hz task + CAN + drive update
bp.rs00.feedback_delay = 0.0035;        % s, CAN feedback + task processing
bp.rs00.command_quantization = 8*pi/(2^16-1); % motion-mode position command
bp.rs00.encoder_quantization = 2*pi/bp.rs00.encoder_counts;
bp.rs00.reflected_motor_inertia = 0.0010; % kg*m^2, placeholder to identify
bp.rs00.joint_viscous_friction = 0.020; % N*m/(rad/s), identify
bp.rs00.joint_coulomb_friction = 0.080; % N*m, identify
bp.rs00.friction_smoothing = 0.05;      % rad/s
bp.rs00.backlash_deadband = 0.05*pi/180; % rad, equivalent small-motion loss

% Two-link closed mechanism:
%
%        A o----------o B==========================o C
%         \             coupler       water pipe     fixed hinge
%          \
%         O o  RS00 fixed output pivot
%
% O and C are fixed in the same x-y frame. A is the end of the RS00 crank;
% B is the attachment point on the pipe. The three moving bodies are the
% motor crank, the coupler and the pipe rocker; it is commonly described as
% a "two-link drive" because crank and coupler transmit motion to the pipe.
%
% OA, AB, BC and the fixed-pivot coordinates below are user-measured. C is
% the right-side hinge and B lies to its left when the pipe is level. The
% 300.1 mm linkage radius is distinct from the 250 mm usable ball path.
bp.mechanism.type = 'four_bar_crank_rocker';
bp.mechanism.motor_axis_height = 0.038;    % m, user-provided
bp.mechanism.pipe_axis_height = 0.093;     % m, user-measured C hinge height
bp.mechanism.fixed_pivot_horizontal_distance = 0.285; % m, O-to-C
bp.mechanism.motor_pivot = [ ...
    -bp.mechanism.fixed_pivot_horizontal_distance; ...
    bp.mechanism.motor_axis_height-bp.mechanism.pipe_axis_height]; % O relative C
bp.mechanism.pipe_pivot = [0.000; 0.000];   % C: water-pipe hinge, m
bp.mechanism.crank_length = 0.0350;         % |OA|, m, user-measured active link
bp.mechanism.coupler_length = 0.0555;       % |AB|, m, user-measured blue link
bp.mechanism.pipe_attachment_radius = 0.3001;% |CB|, m, hinge to blue-link end
bp.mechanism.neutral_B_minus_O_x = ...
    bp.mechanism.fixed_pivot_horizontal_distance ...
    -bp.mechanism.pipe_attachment_radius;   % -15.1 mm: B is left of O
bp.mechanism.assembly_branch = -1;          % physical branch in user's sketch
bp.mechanism.motor_angle_offset = 3.051858578444; % rad; level-pipe solution
bp.mechanism.pipe_angle_offset = pi;        % C-to-B points left at theta=0
bp.mechanism.minimum_jacobian = 0.03;       % avoid toggle/dead-point operation
bp.mechanism.inverse_tolerance = 1e-9;      % rad
bp.mechanism.inverse_iterations = 15;
bp.mechanism.crank_mass = 0.060;            % kg, placeholder until weighed
bp.mechanism.crank_inertia_about_motor = ...
    bp.mechanism.crank_mass*bp.mechanism.crank_length^2/3;
bp.mechanism.coupler_mass = 0.040;          % kg, placeholder until weighed
bp.mechanism.coupler_inertia_about_com = ...
    bp.mechanism.coupler_mass*bp.mechanism.coupler_length^2/12;

% Pipe and bracket inertia about the PIPE pivot. Replace with CAD values.
bp.mechanism.pipe_mass = 0.30;          % kg, PVC pipe + end bracket placeholder
bp.mechanism.pipe_inertia_about_com = ...
    bp.mechanism.pipe_mass*bp.pipe.usable_length^2/12;
bp.mechanism.extra_inertia = 0.001;     % kg*m^2, hinge/end bracket placeholder
% Pipe COM remains provisional; vision x=0 is measured 155 mm from C.
bp.mechanism.com_along_pipe = 0.125;    % m; C to pipe/bracket COM placeholder
bp.mechanism.com_below_pivot = 0.0;     % m; positive gives restoring torque
bp.mechanism.ball_origin_from_pivot = 0.155; % m; C to vision x=0, user-measured
bp.mechanism.pipe_inertia_about_pivot = ...
    bp.mechanism.pipe_inertia_about_com ...
    + bp.mechanism.pipe_mass*( ...
        bp.mechanism.com_along_pipe^2 ...
        + bp.mechanism.com_below_pivot^2) ...
    + bp.mechanism.extra_inertia;

% Lumped RS00 thermal model. These are placeholders until identified from logs.
bp.rs00.thermal.ambient = 25;            % degC
bp.rs00.thermal.initial = 25;            % degC
bp.rs00.thermal.warning = 75;            % degC, official warning threshold
bp.rs00.thermal.cutoff = 80;             % degC, official fault threshold
bp.rs00.thermal.resistance = 1.7;        % degC/W, identify with a load test
bp.rs00.thermal.capacitance = 180;       % J/degC, identify with a load test
bp.rs00.thermal.loss_per_torque_sq = 1.2; % W/(N*m)^2, lumped placeholder

% Pipe-angle safety constraints used by the ball controller. With the
% measured 35/55.5/300.1 mm linkage, |theta| above about 6.5 deg approaches
% the configured Jacobian dead-point guard. Keep the hard command inside
% +/-6 deg and limit pipe rate below the worst-case RS00-speed mapping.
bp.actuator.max_angle = 6.0*pi/180;      % rad, measured-linkage hard command
bp.actuator.max_rate = 0.35;             % pipe rad/s, conservative at +/-6 deg
bp.actuator.delay = bp.rs00.command_delay;

% Camera and controller timing.
bp.sample.Ts_control = 0.005;           % s, 200 Hz controller
bp.sample.Ts_camera = 1/100;            % s, user-confirmed 100 Hz camera
% The WIT UART is 115200 bit/s. Three 11-byte 8N1 frames require 330 bits,
% giving a 349.1-group/s serial ceiling. Model 200 Hz unique groups as the
% deployment baseline; confirm the sensor output rate from source timestamps.
bp.sample.Ts_imu = 1/200;               % s, conservative deployment baseline
bp.sensor.camera_pipeline_delay = 0.025; % s, exposure + Raspberry Pi vision
bp.sensor.rpi_psoc_uart_delay = 0.010;   % s, framing + buffering + processing
bp.sensor.vision_delay = ...
    bp.sensor.camera_pipeline_delay+bp.sensor.rpi_psoc_uart_delay;
bp.sensor.position_sigma = 0.0015;      % m, 1-sigma camera noise
bp.sensor.position_quantization = 0.0005; % m/pixel-equivalent
bp.sensor.dropout_probability = 0.02;   % probability per camera frame
bp.sensor.random_seed = 20260729;
bp.sensor.imu_delay = 0.008;             % s, 2.9 ms wire time + processing/scheduling
bp.sensor.imu_pitch_sigma = 0.10*pi/180; % rad
bp.sensor.imu_accel_sigma = 0.04;        % m/s^2, gravity-compensated axis
bp.sensor.imu_pitch_bias = 0.05*pi/180;  % rad
bp.sensor.imu_accel_bias = 0.02;         % m/s^2

% Vehicle/base-motion disturbance profile. Positive axial acceleration is
% chassis acceleration along +pipe; it creates a -x inertial force on ball.
bp.vehicle.enabled = false;
bp.vehicle.pitch_amplitude = 0.0;        % rad
bp.vehicle.pitch_frequency = 0.0;        % Hz
bp.vehicle.axial_amplitude = 0.0;        % m/s^2
bp.vehicle.axial_frequency = 0.0;        % Hz
bp.vehicle.vertical_amplitude = 0.0;     % m/s^2
bp.vehicle.vertical_frequency = 0.0;     % Hz
% Rows: [start_time, duration, axial_peak, vertical_peak, pitch_peak].
% Each event is a smooth half-sine pulse.
bp.vehicle.events = zeros(0,5);

% Nonlinear plant initial state.
bp.initial.position = 0.020;            % m, safe recovery demonstration
bp.initial.velocity = 0.0;              % m/s
bp.initial.omega = 0.0;                 % rad/s
bp.initial.pipe_angle = 0.0;            % rad
bp.initial.pipe_rate = 0.0;             % rad/s
bp.initial.motor_angle = 0.0;           % rad; matches example linkage at level
bp.initial.motor_rate = NaN;            % rad/s; NaN derives it from pipe rate

% Nominal small-angle model used by the KF and LQI.
% A pure-rolling solid sphere gives x_ddot = (5/7)g*theta.
bp.nominal.acceleration_gain = (5/7) * bp.gravity;
bp.controller.position_limit = ...
    bp.pipe.usable_length/2 - bp.ball.radius;
bp.controller.edge_margin = 0.045;      % m
bp.controller.recovery_angle = 5.5*pi/180; % rad, inside measured workspace
bp.controller.recovery_accel = ...
    bp.nominal.acceleration_gain * bp.controller.recovery_angle;
bp.controller.integral_limit = 0.25;    % m*s
bp.controller.innovation_gate_sigma = 6.0;
bp.controller.actuator_tracking_tolerance = 0.5*pi/180; % rad
bp.controller.vehicle_feedforward_enabled = true;
bp.controller.disturbance_observer_enabled = true;
bp.controller.disturbance_compensation_limit = 2.5*pi/180; % rad

% LQI weights use physically meaningful "acceptable maximum" scales.
bp.controller.design.position_scale = 0.040; % m
bp.controller.design.velocity_scale = 0.35;  % m/s
bp.controller.design.integral_scale = 0.12;  % m*s
bp.controller.design.angle_scale = 5.5*pi/180; % rad
bp.controller.K = local_design_lqi(bp);

% Kalman-filter covariance.
bp.kalman.acceleration_sigma = 0.80;    % m/s^2, unmodelled acceleration
bp.kalman.initial_position_sigma = 0.030; % m
bp.kalman.initial_velocity_sigma = 0.30;  % m/s
bp.kalman.initial_disturbance_sigma = 0.20; % m/s^2
bp.kalman.disturbance_random_walk_sigma = 0.30; % m/s^3

% Reference and simulation.
bp.reference.time = [0; 2; 5; 8; 11; 14];
bp.reference.position = [0; 0; 0.020; -0.020; 0; 0];
bp.sim.stop_time = 14.0;
bp.sim.fixed_step = 1/3000;             % s; common grid for 100/200/500 Hz tasks

end


function K = local_design_lqi(bp)
% Solve the discrete algebraic Riccati equation without Control System Toolbox.
Ts = bp.sample.Ts_control;
b = bp.nominal.acceleration_gain;

A = [1, Ts; 0, 1];
B = [0.5*b*Ts^2; b*Ts];

% xi(k+1) = xi(k) + Ts*(x(k)-r(k))
Aa = [A, zeros(2,1); Ts, 0, 1];
Ba = [B; 0];

Q = diag([ ...
    1/bp.controller.design.position_scale^2, ...
    1/bp.controller.design.velocity_scale^2, ...
    1/bp.controller.design.integral_scale^2]);
R = 1/bp.controller.design.angle_scale^2;

P = Q;
for k = 1:20000
    denominator = R + Ba' * P * Ba;
    Pnext = Aa' * P * Aa ...
        - (Aa' * P * Ba) * (Ba' * P * Aa) / denominator + Q;
    if norm(Pnext-P, 'fro') <= 1e-11 * max(1, norm(P, 'fro'))
        P = Pnext;
        break;
    end
    P = Pnext;
end

K = (R + Ba' * P * Ba) \ (Ba' * P * Aa);

end
