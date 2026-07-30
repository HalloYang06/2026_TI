function [pipe_angle, jacobian, jacobian_derivative, valid] = ...
    fourbar_kinematics(motor_angle, bp, calculate_derivative)
%FOURBAR_KINEMATICS Forward kinematics of the RS00-crank-coupler-pipe linkage.
%
% Inputs:
%   motor_angle  RS00 output-shaft angle q, rad
%   bp           structure from ball_pipe_defaults
%
% Outputs:
%   pipe_angle          pipe rocker angle theta, rad
%   jacobian            d(theta)/d(q)
%   jacobian_derivative d(jacobian)/d(q)
%   valid               true when the requested assembly can be constructed

if nargin < 3
    calculate_derivative = true;
end

[pipe_angle, jacobian, valid] = local_angle_and_jacobian(motor_angle, bp);

if valid && calculate_derivative
    step = 1e-5;
    [~, jacobian_plus, valid_plus] = ...
        local_angle_and_jacobian(motor_angle+step, bp);
    [~, jacobian_minus, valid_minus] = ...
        local_angle_and_jacobian(motor_angle-step, bp);
    if valid_plus && valid_minus
        jacobian_derivative = (jacobian_plus-jacobian_minus)/(2*step);
    else
        jacobian_derivative = 0;
    end
else
    jacobian_derivative = 0;
end

end


function [pipe_angle, jacobian, valid] = ...
    local_angle_and_jacobian(motor_angle, bp)
mechanism = bp.mechanism;
q = motor_angle+mechanism.motor_angle_offset;

O = mechanism.motor_pivot(:);
C = mechanism.pipe_pivot(:);
a = mechanism.crank_length;
b = mechanism.coupler_length;
c = mechanism.pipe_attachment_radius;

A = O+a*[cos(q); sin(q)];
CA = A-C;
distance = hypot(CA(1), CA(2));

valid = distance > eps ...
    && distance <= b+c ...
    && distance >= abs(b-c);
if ~valid
    pipe_angle = 0;
    jacobian = 0;
    return;
end

along = (c^2-b^2+distance^2)/(2*distance);
height_squared = c^2-along^2;
if height_squared < -1e-12
    pipe_angle = 0;
    jacobian = 0;
    valid = false;
    return;
end
height = sqrt(max(height_squared, 0));

unit_ca = CA/distance;
normal_ca = [-unit_ca(2); unit_ca(1)];
B = C+along*unit_ca ...
    + mechanism.assembly_branch*height*normal_ca;

raw_pipe_angle = atan2(B(2)-C(2), B(1)-C(1));
pipe_angle = local_wrap(raw_pipe_angle-mechanism.pipe_angle_offset);

coupler_vector = B-A;
dA_dq = a*[-sin(q); cos(q)];
dB_dtheta = c*[-sin(raw_pipe_angle); cos(raw_pipe_angle)];
denominator = dot(coupler_vector, dB_dtheta);
if abs(denominator) <= 1e-10
    jacobian = 0;
    valid = false;
    return;
end

jacobian = dot(coupler_vector, dA_dq)/denominator;
valid = isfinite(jacobian) ...
    && abs(jacobian) >= mechanism.minimum_jacobian;

end


function wrapped = local_wrap(angle)
wrapped = atan2(sin(angle), cos(angle));

end
