function [motor_angle, valid] = ...
    fourbar_inverse_kinematics(pipe_angle_target, initial_guess, bp)
%FOURBAR_INVERSE_KINEMATICS Solve q for a desired pipe angle.
%
% Newton iteration starts from the current motor angle, which keeps the
% solution on the physical assembly branch during continuous operation.

motor_angle = initial_guess;
valid = true;

for iteration = 1:bp.mechanism.inverse_iterations
    [pipe_angle, jacobian, ~, kinematics_valid] = ...
        fourbar_kinematics(motor_angle, bp, false);
    if ~kinematics_valid ...
            || abs(jacobian) < bp.mechanism.minimum_jacobian
        valid = false;
        return;
    end

    error = atan2( ...
        sin(pipe_angle_target-pipe_angle), ...
        cos(pipe_angle_target-pipe_angle));
    if abs(error) <= bp.mechanism.inverse_tolerance
        return;
    end

    step = error/jacobian;
    step = min(max(step, -0.25), 0.25);
    motor_angle = motor_angle+step;
end

[final_angle, ~, ~, kinematics_valid] = ...
    fourbar_kinematics(motor_angle, bp, false);
final_error = atan2( ...
    sin(pipe_angle_target-final_angle), ...
    cos(pipe_angle_target-final_angle));
valid = kinematics_valid ...
    && abs(final_error) <= 10*bp.mechanism.inverse_tolerance;

end
