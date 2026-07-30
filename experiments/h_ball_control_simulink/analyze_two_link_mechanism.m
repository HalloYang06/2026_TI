%ANALYZE_TWO_LINK_MECHANISM Plot workspace, Jacobian and reflected inertia.

clearvars;
close all;

root = fileparts(mfilename('fullpath'));
addpath(root);
bp = ball_pipe_defaults();

pipe_angle = linspace( ...
    -bp.actuator.max_angle, bp.actuator.max_angle, 161);
motor_angle = nan(size(pipe_angle));
jacobian = nan(size(pipe_angle));
equivalent_inertia = nan(size(pipe_angle));
valid = false(size(pipe_angle));

guess = bp.initial.motor_angle;
for index = 1:numel(pipe_angle)
    [motor_angle(index), inverse_valid] = ...
        fourbar_inverse_kinematics(pipe_angle(index), guess, bp);
    [~, jacobian(index), ~, forward_valid] = ...
        fourbar_kinematics(motor_angle(index), bp, false);
    [equivalent_inertia(index), ~, inertia_valid] = ...
        fourbar_equivalent_inertia(motor_angle(index), bp);
    valid(index) = inverse_valid && forward_valid && inertia_valid;
    if valid(index)
        guess = motor_angle(index);
    end
end

if ~all(valid)
    warning(['The requested pipe-angle range is not fully reachable. ' ...
        'Check linkage dimensions, assembly branch and angle offsets.']);
end

torque_amplification = abs(1./jacobian);
motor_speed_for_unit_pipe_speed = abs(1./jacobian);

fprintf('\nTwo-link mechanism analysis\n');
fprintf('  Requested pipe range       : [%.2f, %.2f] deg\n', ...
    min(pipe_angle)*180/pi, max(pipe_angle)*180/pi);
fprintf('  Required motor range       : [%.2f, %.2f] deg\n', ...
    min(motor_angle(valid))*180/pi, max(motor_angle(valid))*180/pi);
fprintf('  Minimum |dtheta/dq|        : %.4f\n', ...
    min(abs(jacobian(valid))));
fprintf('  Maximum torque amplification: %.2f pipe-Nm / motor-Nm\n', ...
    max(torque_amplification(valid)));
fprintf('  Unit pipe-rate needs up to : %.2f rad/s motor speed\n', ...
    max(motor_speed_for_unit_pipe_speed(valid)));
fprintf('  Pipe rate at motor limit   : %.3f rad/s worst-case\n', ...
    bp.rs00.user_speed_limit*min(abs(jacobian(valid))));
fprintf('  Configured pipe rate limit : %.3f rad/s\n', ...
    bp.actuator.max_rate);
fprintf('  All requested poses valid  : %s\n\n', string(all(valid)));

figure('Color', 'w', 'Name', 'RS00 two-link mechanism analysis');
tiledlayout(2,2, 'TileSpacing', 'compact');

nexttile;
hold on;
configuration_indices = [1, ceil(numel(pipe_angle)/2), numel(pipe_angle)];
configuration_colours = lines(numel(configuration_indices));
for plot_index = 1:numel(configuration_indices)
    index = configuration_indices(plot_index);
    [O, A, B, C] = local_linkage_points( ...
        motor_angle(index), pipe_angle(index), bp);
    colour = configuration_colours(plot_index,:);
    plot([O(1), A(1), B(1), C(1)], ...
        [O(2), A(2), B(2), C(2)], '-o', ...
        'Color', colour, 'LineWidth', 1.5, ...
        'DisplayName', sprintf('\\theta = %.1f deg', ...
        pipe_angle(index)*180/pi));
    raw_pipe_angle = ...
        pipe_angle(index)+bp.mechanism.pipe_angle_offset;
    pipe_direction = [cos(raw_pipe_angle); sin(raw_pipe_angle)];
    plot([C(1), C(1)+bp.pipe.usable_length*pipe_direction(1)], ...
        [C(2), C(2)+bp.pipe.usable_length*pipe_direction(2)], ...
        '-', 'Color', colour, 'LineWidth', 3, ...
        'HandleVisibility', 'off');
end
axis equal;
grid on;
xlabel('x (m)');
ylabel('y (m)');
title('Example linkage poses');
legend('Location', 'best');

nexttile;
plot(pipe_angle(valid)*180/pi, motor_angle(valid)*180/pi, ...
    'b', 'LineWidth', 1.5);
grid on;
xlabel('Pipe angle \theta (deg)');
ylabel('RS00 angle q (deg)');
title('Inverse kinematics');

nexttile;
yyaxis left;
plot(pipe_angle(valid)*180/pi, abs(jacobian(valid)), ...
    'b', 'LineWidth', 1.5);
yline(bp.mechanism.minimum_jacobian, 'r--');
ylabel('|d\theta/dq|');
yyaxis right;
plot(pipe_angle(valid)*180/pi, torque_amplification(valid), ...
    'Color', [0.85, 0.35, 0.1], 'LineWidth', 1.5);
ylabel('|\tau_{pipe}/\tau_{motor}|');
grid on;
xlabel('Pipe angle \theta (deg)');
title('Speed ratio and torque amplification');

nexttile;
plot(pipe_angle(valid)*180/pi, ...
    1e3*equivalent_inertia(valid), 'Color', [0.35, 0.2, 0.65], ...
    'LineWidth', 1.5);
grid on;
xlabel('Pipe angle \theta (deg)');
ylabel('Equivalent motor inertia (10^{-3} kg m^2)');
title('Configuration-dependent reflected inertia');

result_path = fullfile(root, 'two_link_mechanism_analysis.png');
exportgraphics(gcf, result_path, 'Resolution', 160);
fprintf('Saved mechanism plot: %s\n', result_path);


function [O, A, B, C] = local_linkage_points(motor_angle, pipe_angle, bp)
mechanism = bp.mechanism;
q = motor_angle+mechanism.motor_angle_offset;
theta = pipe_angle+mechanism.pipe_angle_offset;
O = mechanism.motor_pivot(:);
C = mechanism.pipe_pivot(:);
A = O+mechanism.crank_length*[cos(q); sin(q)];
B = C+mechanism.pipe_attachment_radius*[cos(theta); sin(theta)];

end
