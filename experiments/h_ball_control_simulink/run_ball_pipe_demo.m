%RUN_BALL_PIPE_DEMO Build, simulate and plot the nominal closed-loop model.

clearvars;
close all;

root = fileparts(mfilename('fullpath'));
addpath(root);

bp = ball_pipe_defaults();
[model_name, model_path] = build_ball_pipe_model(bp);

simulation_output = sim(model_name, ...
    'StopTime', num2str(bp.sim.stop_time, 16), ...
    'ReturnWorkspaceOutputs', 'on');

ball = simulation_output.get('sim_ball');
rs00 = simulation_output.get('sim_rs00');
camera = simulation_output.get('sim_camera');
controller = simulation_output.get('sim_controller');
reference = simulation_output.get('sim_reference');

reference_at_ball_time = interp1(reference.Time, reference.Data, ...
    ball.Time, 'previous', 'extrap');
position_error = ball.Data(:,1)-reference_at_ball_time;
rms_error = sqrt(mean(position_error.^2));
maximum_error = max(abs(position_error));
dropped = any(ball.Data(:,6) > 0.5);
linkage_valid = all(rs00.Data(:,8) > 0.5);
minimum_abs_jacobian = min(abs(rs00.Data(:,7)));
maximum_motor_angle = max(abs(rs00.Data(:,10)));
maximum_motor_rate = max(abs(rs00.Data(:,11)));

fprintf('\nNominal simulation metrics\n');
fprintf('  RMS position error : %.3f mm\n', 1e3*rms_error);
fprintf('  Maximum error      : %.3f mm\n', 1e3*maximum_error);
fprintf('  Ball dropped       : %s\n', string(dropped));
fprintf('  Linkage valid      : %s\n', string(linkage_valid));
fprintf('  Min |dtheta/dq|    : %.4f\n', minimum_abs_jacobian);
fprintf('  Max motor angle    : %.3f deg\n', maximum_motor_angle*180/pi);
fprintf('  Max motor rate     : %.3f rad/s\n', maximum_motor_rate);
fprintf('  LQI gain [Kx Kv Ki]: [%g %g %g]\n', bp.controller.K);
fprintf('  Model              : %s\n\n', model_path);

figure('Color', 'w', 'Name', 'Steel ball / PVC pipe simulation');
tiledlayout(4,1, 'TileSpacing', 'compact');

nexttile;
plot(reference.Time, 1e3*reference.Data, 'k--', 'LineWidth', 1.2);
hold on;
plot(ball.Time, 1e3*ball.Data(:,1), 'b', 'LineWidth', 1.3);
plot(controller.Time, 1e3*controller.Data(:,2), 'r:', 'LineWidth', 1.1);
yline(1e3*bp.controller.position_limit, 'Color', [0.7 0.2 0.2]);
yline(-1e3*bp.controller.position_limit, 'Color', [0.7 0.2 0.2]);
grid on;
ylabel('Position (mm)');
legend('Reference', 'True', 'KF estimate', 'Pipe ends', ...
    'Location', 'best');

nexttile;
plot(ball.Time, ball.Data(:,2), 'b', 'LineWidth', 1.2);
hold on;
plot(controller.Time, controller.Data(:,3), 'r:', 'LineWidth', 1.1);
grid on;
ylabel('Velocity (m/s)');
legend('True', 'KF estimate', 'Location', 'best');

nexttile;
plot(rs00.Time, rs00.Data(:,1)*180/pi, 'b', 'LineWidth', 1.2);
hold on;
plot(controller.Time, controller.Data(:,1)*180/pi, 'r--', 'LineWidth', 1.0);
grid on;
ylabel('Pipe angle (deg)');
legend('Actual', 'Command', 'Location', 'best');

nexttile;
yyaxis left;
plot(ball.Time, 1e3*ball.Data(:,5), 'Color', [0.4 0.2 0.7], ...
    'LineWidth', 1.1);
hold on;
stairs(camera.Time, camera.Data(:,3), 'Color', [0.2 0.6 0.2]);
stairs(controller.Time, controller.Data(:,4), 'Color', [0.8 0.2 0.2]);
ylabel('Slip (mm/s) / flags');
yyaxis right;
plot(rs00.Time, rs00.Data(:,3), 'Color', [0.1 0.5 0.8], ...
    'LineWidth', 1.0);
plot(rs00.Time, rs00.Data(:,5), 'Color', [0.8 0.4 0.1], ...
    'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('RS00 torque (N m) / temperature (degC)');
legend('Ball-pipe slip', 'Camera valid', 'Edge recovery', ...
    'RS00 torque', 'RS00 temperature', ...
    'Location', 'best');

result_path = fullfile(root, 'ball_pipe_nominal_result.png');
exportgraphics(gcf, result_path, 'Resolution', 160);
fprintf('Saved result plot: %s\n', result_path);

open_system(model_name);
