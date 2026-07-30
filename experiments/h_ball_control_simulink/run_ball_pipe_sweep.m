%RUN_BALL_PIPE_SWEEP Compare friction, delay and resistance uncertainty.

clearvars;
close all;

root = fileparts(mfilename('fullpath'));
addpath(root);

nominal = ball_pipe_defaults();
[model_name, ~] = build_ball_pipe_model(nominal);

scenario_names = { ...
    'Nominal', ...
    'Low steel-PVC friction', ...
    'High steel-PVC friction', ...
    'Large visual/actuator delay', ...
    'High rolling resistance', ...
    'Worst combined case'};

scenarios = repmat(nominal, 1, numel(scenario_names));

scenarios(2).contact.mu_static = 0.10;
scenarios(2).contact.mu_kinetic = 0.07;

scenarios(3).contact.mu_static = 0.48;
scenarios(3).contact.mu_kinetic = 0.36;
scenarios(3).contact.slip_viscous = 0.025;

scenarios(4).sensor.vision_delay = 0.085;
scenarios(4).rs00.command_delay = 0.020;
scenarios(4).actuator.delay = scenarios(4).rs00.command_delay;

scenarios(5).contact.rolling_resistance = 0.025;
scenarios(5).drag.viscous = 0.025;

scenarios(6).contact.mu_static = 0.10;
scenarios(6).contact.mu_kinetic = 0.07;
scenarios(6).sensor.vision_delay = 0.085;
scenarios(6).rs00.command_delay = 0.020;
scenarios(6).actuator.delay = scenarios(6).rs00.command_delay;
scenarios(6).sensor.dropout_probability = 0.10;

results = struct([]);
figure('Color', 'w', 'Name', 'Ball-pipe robustness sweep');
tiledlayout(2,1, 'TileSpacing', 'compact');

for index = 1:numel(scenarios)
    bp = scenarios(index); %#ok<NASGU>
    assignin('base', 'bp', bp);

    simulation_output = sim(model_name, ...
        'StopTime', num2str(bp.sim.stop_time, 16), ...
        'ReturnWorkspaceOutputs', 'on');

    ball = simulation_output.get('sim_ball');
    reference = simulation_output.get('sim_reference');
    controller = simulation_output.get('sim_controller');

    reference_at_ball_time = interp1(reference.Time, reference.Data, ...
        ball.Time, 'previous', 'extrap');
    error = ball.Data(:,1)-reference_at_ball_time;

    results(index).name = scenario_names{index}; %#ok<SAGROW>
    results(index).rms_mm = 1e3*sqrt(mean(error.^2));
    results(index).max_mm = 1e3*max(abs(error));
    results(index).dropped = any(ball.Data(:,6) > 0.5);
    results(index).edge_recovery_count = ...
        sum(diff(controller.Data(:,4) > 0.5) == 1);

    nexttile(1);
    hold on;
    plot(ball.Time, 1e3*ball.Data(:,1), 'LineWidth', 1.0, ...
        'DisplayName', scenario_names{index});

    nexttile(2);
    hold on;
    plot(ball.Time, 1e3*ball.Data(:,5), 'LineWidth', 1.0, ...
        'DisplayName', scenario_names{index});
end

nexttile(1);
reference = simulation_output.get('sim_reference');
plot(reference.Time, 1e3*reference.Data, 'k--', 'LineWidth', 1.4, ...
    'DisplayName', 'Reference');
grid on;
ylabel('Ball position (mm)');
legend('Location', 'eastoutside');

nexttile(2);
grid on;
xlabel('Time (s)');
ylabel('Slip velocity (mm/s)');
legend('Location', 'eastoutside');

fprintf('\nRobustness sweep\n');
fprintf('%-29s %10s %10s %9s %10s\n', ...
    'Scenario', 'RMS/mm', 'Max/mm', 'Dropped', 'Recoveries');
for index = 1:numel(results)
    fprintf('%-29s %10.2f %10.2f %9s %10d\n', ...
        results(index).name, results(index).rms_mm, ...
        results(index).max_mm, string(results(index).dropped), ...
        results(index).edge_recovery_count);
end

result_path = fullfile(root, 'ball_pipe_robustness_sweep.png');
exportgraphics(gcf, result_path, 'Resolution', 160);
fprintf('\nSaved sweep plot: %s\n', result_path);
