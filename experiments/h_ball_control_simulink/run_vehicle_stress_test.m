%RUN_VEHICLE_STRESS_TEST Mobile-platform low-friction pressure test.

clearvars -except vehicle_stress_selection;
close all;

root = fileparts(mfilename('fullpath'));
addpath(root);

base = ball_pipe_defaults();
base.sim.stop_time = 5.0;
base.initial.position = 0.025;
base.initial.velocity = 0;
base.reference.time = [0; base.sim.stop_time];
base.reference.position = [0; 0];
base.controller.edge_margin = 0.035;
minimum_accepted_margin = 0.025;       % m
maximum_accepted_slip = 0.150;         % m/s

% Provisional envelope for the user's very slippery steel/PVC measurement.
base.contact.mu_static = 0.040;
base.contact.mu_kinetic = 0.025;
base.contact.rolling_resistance = 0.0010;

scenario_names = { ...
    'Low friction, stationary', ...
    'Normal vehicle motion', ...
    'Acceleration and hard brake', ...
    'Pothole plus degraded UART vision', ...
    'Worst combined, optimized', ...
    'Worst combined, KF-LQI only', ...
    '3.0 m/s2 transient over steady authority'};

scenarios = repmat(base, 1, numel(scenario_names));

% Smooth road and normal start/stop motion.
scenarios(2).vehicle.enabled = true;
scenarios(2).vehicle.pitch_amplitude = 1.0*pi/180;
scenarios(2).vehicle.pitch_frequency = 0.35;
scenarios(2).vehicle.axial_amplitude = 0.45;
scenarios(2).vehicle.axial_frequency = 0.40;
scenarios(2).vehicle.vertical_amplitude = 0.20;
scenarios(2).vehicle.vertical_frequency = 8.0;

% Start followed by a substantial brake event.
scenarios(3) = scenarios(2);
scenarios(3).vehicle.events = [ ...
    0.8, 0.60,  0.90, 0.20,  2.0*pi/180; ...
    2.8, 0.75, -1.50, 0.30, -3.0*pi/180];

% Rough road, larger Raspberry-Pi processing/UART delay and packet loss.
scenarios(4) = scenarios(2);
scenarios(4).vehicle.vertical_amplitude = 0.55;
scenarios(4).vehicle.vertical_frequency = 9.0;
scenarios(4).vehicle.events = [ ...
    1.7, 0.16, 0.65, 3.5, 1.5*pi/180; ...
    3.6, 0.12, -0.55, -2.5, -1.0*pi/180];
scenarios(4).sensor.vision_delay = 0.100;
scenarios(4).sensor.dropout_probability = 0.25;
scenarios(4).sensor.position_sigma = 0.0030;

% Very low friction, brake/pothole, camera/UART degradation and IMU bias.
scenarios(5) = scenarios(4);
scenarios(5).contact.mu_static = 0.020;
scenarios(5).contact.mu_kinetic = 0.012;
scenarios(5).contact.rolling_resistance = 0.0005;
scenarios(5).vehicle.axial_amplitude = 0.65;
scenarios(5).vehicle.events = [ ...
    0.9, 0.55,  1.10, 0.5,  2.0*pi/180; ...
    2.6, 0.80, -1.80, 3.0, -3.5*pi/180];
scenarios(5).sensor.imu_pitch_bias = 0.25*pi/180;
scenarios(5).sensor.imu_accel_bias = 0.08;
scenarios(5).rs00.command_delay = 0.008;
scenarios(5).actuator.delay = scenarios(5).rs00.command_delay;

% Direct algorithm comparison on exactly the same worst-case disturbance.
scenarios(6) = scenarios(5);
scenarios(6).controller.vehicle_feedforward_enabled = false;
scenarios(6).controller.disturbance_observer_enabled = false;

% Deliberately exceeds g*sin(8 deg) compensation authority.
scenarios(7) = scenarios(5);
scenarios(7).vehicle.events = [ ...
    2.0, 0.90, -3.00, 1.0, -4.0*pi/180];
scenarios(7).sensor.vision_delay = 0.050;
scenarios(7).sensor.dropout_probability = 0.05;

if exist('vehicle_stress_selection', 'var') ...
        && ~isempty(vehicle_stress_selection)
    scenario_names = scenario_names(vehicle_stress_selection);
    scenarios = scenarios(vehicle_stress_selection);
end

[model_name, ~] = build_ball_pipe_model(base);

results = struct([]);
figure('Color', 'w', 'Name', 'Vehicle-mounted ball-pipe stress test');
tiledlayout(3,1, 'TileSpacing', 'compact');
colours = lines(numel(scenarios));

for index = 1:numel(scenarios)
    bp = scenarios(index); %#ok<NASGU>
    assignin('base', 'bp', bp);
    reference = timeseries(bp.reference.position, bp.reference.time);
    assignin('base', 'bp_reference', reference);

    simulation_output = sim(model_name, ...
        'StopTime', num2str(bp.sim.stop_time, 16), ...
        'ReturnWorkspaceOutputs', 'on');

    ball = simulation_output.get('sim_ball');
    rs00 = simulation_output.get('sim_rs00');
    camera = simulation_output.get('sim_camera');
    controller = simulation_output.get('sim_controller');

    position = ball.Data(:,1);
    dropped = any(ball.Data(:,6) > 0.5);
    linkage_valid = all(rs00.Data(:,8) > 0.5);
    position_limit = bp.controller.position_limit;
    maximum_position = max(abs(position));

    results(index).name = scenario_names{index}; %#ok<SAGROW>
    results(index).rms_mm = 1e3*sqrt(mean(position.^2));
    results(index).max_mm = 1e3*maximum_position;
    results(index).minimum_margin_mm = ...
        1e3*(position_limit-maximum_position);
    results(index).dropped = dropped;
    results(index).linkage_valid = linkage_valid;
    results(index).max_motor_torque = max(abs(rs00.Data(:,3)));
    results(index).max_motor_rate = max(abs(rs00.Data(:,11)));
    maximum_slip = max(abs(ball.Data(:,5)));
    results(index).max_slip_mm_s = 1e3*maximum_slip;
    results(index).camera_valid_fraction = mean(camera.Data(:,3) > 0.5);
    results(index).edge_recovery_fraction = ...
        mean(controller.Data(:,4) > 0.5);
    results(index).passed = ~dropped && linkage_valid ...
        && position_limit-maximum_position >= minimum_accepted_margin ...
        && maximum_slip <= maximum_accepted_slip;

    nexttile(1);
    hold on;
    plot(ball.Time, 1e3*position, 'Color', colours(index,:), ...
        'LineWidth', 1.0, 'DisplayName', scenario_names{index});

    nexttile(2);
    hold on;
    plot(ball.Time, ball.Data(:,8), 'Color', colours(index,:), ...
        'LineWidth', 1.0, 'DisplayName', scenario_names{index});

    nexttile(3);
    hold on;
    plot(rs00.Time, rs00.Data(:,1)*180/pi, ...
        'Color', colours(index,:), 'LineWidth', 1.0, ...
        'DisplayName', scenario_names{index});
end

nexttile(1);
yline(1e3*base.controller.position_limit, 'r--');
yline(-1e3*base.controller.position_limit, 'r--');
grid on;
ylabel('Ball position (mm)');
legend('Location', 'eastoutside');

nexttile(2);
grid on;
ylabel('Vehicle axial a (m/s^2)');

nexttile(3);
grid on;
xlabel('Time (s)');
ylabel('Pipe angle (deg)');

fprintf('\nVehicle-mounted low-friction stress test\n');
fprintf('%-38s %8s %8s %9s %7s %8s %8s\n', ...
    'Scenario', 'RMS/mm', 'Max/mm', 'Margin/mm', 'Pass', 'T/Nm', 'qdot');
for index = 1:numel(results)
    fprintf('%-38s %8.2f %8.2f %9.2f %7s %8.2f %8.2f\n', ...
        results(index).name, results(index).rms_mm, ...
        results(index).max_mm, results(index).minimum_margin_mm, ...
        string(results(index).passed), results(index).max_motor_torque, ...
        results(index).max_motor_rate);
end

result_path = fullfile(root, 'vehicle_stress_test.png');
exportgraphics(gcf, result_path, 'Resolution', 160);
save(fullfile(root, 'vehicle_stress_results.mat'), ...
    'results', 'scenario_names', 'scenarios');
fprintf('\nSaved stress plot: %s\n', result_path);
