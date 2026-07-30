%RUN_CAMERA_RATE_COMPARISON Compare 30, 60 and 100 Hz camera feedback.
%
% The test starts inside the 10 mm accuracy band at +5 mm. It compares
% centre holding on a moving vehicle and a bounded start/brake/pothole case.
% The estimator/controller remain at 200 Hz; the current WIT source model
% supplies about 30 unique IMU groups/s and holds each sample between updates.

clearvars;
close all;

root = fileparts(mfilename('fullpath'));
addpath(root);

camera_rates_hz = [100, 60, 30];
scenario_names = {'Normal moving vehicle', 'Start, brake and pothole'};
results = struct([]);

base = ball_pipe_defaults();
base.sim.stop_time = 5.0;
base.initial.position = 0.005;
base.initial.velocity = 0;
base.reference.time = [0; base.sim.stop_time];
base.reference.position = [0; 0];
base.contact.mu_static = 0.040;
base.contact.mu_kinetic = 0.025;
base.contact.rolling_resistance = 0.0010;
base.sensor.dropout_probability = 0.02;
base.vehicle.enabled = true;
base.vehicle.pitch_amplitude = 1.0*pi/180;
base.vehicle.pitch_frequency = 0.35;
base.vehicle.axial_amplitude = 0.45;
base.vehicle.axial_frequency = 0.40;
base.vehicle.vertical_amplitude = 0.20;
base.vehicle.vertical_frequency = 8.0;

scenarios = repmat(base, 1, numel(scenario_names));
scenarios(2).vehicle.events = [ ...
    0.8, 0.55,  0.90, 0.5,  2.0*pi/180; ...
    2.8, 0.65, -1.20, 1.5, -2.5*pi/180];

[model_name, ~] = build_ball_pipe_model(base);

figure('Color', 'w', 'Name', 'Camera-rate comparison');
tiledlayout(2,1, 'TileSpacing', 'compact');
colours = lines(numel(camera_rates_hz)*numel(scenarios));
result_index = 0;

for scenario_index = 1:numel(scenarios)
    for rate_index = 1:numel(camera_rates_hz)
        result_index = result_index+1;
        bp = scenarios(scenario_index); %#ok<NASGU>
        bp.sample.Ts_camera = 1/camera_rates_hz(rate_index);
        assignin('base', 'bp', bp);
        assignin('base', 'bp_reference', ...
            timeseries(bp.reference.position, bp.reference.time));

        simulation_output = sim(model_name, ...
            'StopTime', num2str(bp.sim.stop_time, 16), ...
            'ReturnWorkspaceOutputs', 'on');

        ball = simulation_output.get('sim_ball');
        camera = simulation_output.get('sim_camera');
        position = ball.Data(:,1);
        error = position;
        peak_error = max(abs(error));

        results(result_index).scenario = scenario_names{scenario_index}; %#ok<SAGROW>
        results(result_index).camera_rate_hz = camera_rates_hz(rate_index);
        results(result_index).rms_mm = 1e3*sqrt(mean(error.^2));
        results(result_index).peak_mm = 1e3*peak_error;
        results(result_index).within_10mm_fraction = ...
            mean(abs(error) <= 0.010);
        results(result_index).camera_valid_fraction = ...
            mean(camera.Data(:,3) > 0.5);
        results(result_index).dropped = any(ball.Data(:,6) > 0.5);
        results(result_index).strict_10mm_pass = ...
            peak_error <= 0.010 && ~results(result_index).dropped;

        nexttile(scenario_index);
        hold on;
        plot(ball.Time, 1e3*error, ...
            'Color', colours(result_index,:), 'LineWidth', 1.2, ...
            'DisplayName', sprintf('%g Hz', camera_rates_hz(rate_index)));
    end
end

for tile_index = 1:numel(scenarios)
    nexttile(tile_index);
    yline(10, 'r--', '10 mm limit', 'HandleVisibility', 'off');
    yline(-10, 'r--', '-10 mm limit', 'HandleVisibility', 'off');
    grid on;
    ylabel('Position error (mm)');
    title(scenario_names{tile_index});
    legend('Location', 'best');
end
xlabel('Time (s)');

fprintf('\nCamera-rate comparison (initial error = 5 mm)\n');
fprintf('%-27s %7s %9s %9s %10s %8s\n', ...
    'Scenario', 'Hz', 'RMS/mm', 'Peak/mm', 'In band', 'Pass');
for index = 1:numel(results)
    fprintf('%-27s %7.0f %9.3f %9.3f %9.2f%% %8s\n', ...
        results(index).scenario, results(index).camera_rate_hz, ...
        results(index).rms_mm, results(index).peak_mm, ...
        100*results(index).within_10mm_fraction, ...
        string(results(index).strict_10mm_pass));
end

plot_path = fullfile(root, 'camera_rate_comparison.png');
exportgraphics(gcf, plot_path, 'Resolution', 180);
save(fullfile(root, 'camera_rate_comparison_results.mat'), ...
    'results', 'camera_rates_hz', 'scenario_names', 'scenarios');
fprintf('\nSaved comparison plot: %s\n', plot_path);
