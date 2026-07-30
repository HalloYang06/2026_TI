function [equivalent_inertia, inertia_derivative, valid] = ...
    fourbar_equivalent_inertia(motor_angle, bp)
%FOURBAR_EQUIVALENT_INERTIA Configuration-dependent inertia at RS00 shaft.
%
% Includes the RS00 reflected inertia, crank, coupler translation/rotation,
% and the pipe rocker reflected through d(theta)/d(q).

[equivalent_inertia, valid] = local_inertia(motor_angle, bp);

if valid
    step = 1e-5;
    [inertia_plus, valid_plus] = local_inertia(motor_angle+step, bp);
    [inertia_minus, valid_minus] = local_inertia(motor_angle-step, bp);
    if valid_plus && valid_minus
        inertia_derivative = (inertia_plus-inertia_minus)/(2*step);
    else
        inertia_derivative = 0;
    end
else
    inertia_derivative = 0;
end

end


function [equivalent_inertia, valid] = local_inertia(motor_angle, bp)
[pipe_angle, jacobian, ~, valid] = ...
    fourbar_kinematics(motor_angle, bp, false);
if ~valid
    equivalent_inertia = bp.rs00.reflected_motor_inertia;
    return;
end

mechanism = bp.mechanism;
q = motor_angle+mechanism.motor_angle_offset;
raw_pipe_angle = pipe_angle+mechanism.pipe_angle_offset;

dA_dq = mechanism.crank_length*[-sin(q); cos(q)];
dB_dq = mechanism.pipe_attachment_radius ...
    * [-sin(raw_pipe_angle); cos(raw_pipe_angle)]*jacobian;
dCouplerCom_dq = 0.5*(dA_dq+dB_dq);

O = mechanism.motor_pivot(:);
C = mechanism.pipe_pivot(:);
A = O+mechanism.crank_length*[cos(q); sin(q)];
B = C+mechanism.pipe_attachment_radius ...
    * [cos(raw_pipe_angle); sin(raw_pipe_angle)];
coupler = B-A;
dCoupler_dq = dB_dq-dA_dq;
coupler_angle_jacobian = ...
    (coupler(1)*dCoupler_dq(2)-coupler(2)*dCoupler_dq(1)) ...
    /max(dot(coupler, coupler), eps);

equivalent_inertia = ...
    bp.rs00.reflected_motor_inertia ...
    + mechanism.crank_inertia_about_motor ...
    + mechanism.coupler_mass*dot(dCouplerCom_dq, dCouplerCom_dq) ...
    + mechanism.coupler_inertia_about_com*coupler_angle_jacobian^2 ...
    + mechanism.pipe_inertia_about_pivot*jacobian^2;
valid = isfinite(equivalent_inertia) && equivalent_inertia > 0;

end
