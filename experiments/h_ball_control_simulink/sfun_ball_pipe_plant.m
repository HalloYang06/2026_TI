function sfun_ball_pipe_plant(block)
%SFUN_BALL_PIPE_PLANT Nonlinear rolling/sliding steel-ball pipe plant.
%
% Input:
%   u(1) - actual pipe angle, rad
%
% Output:
%   [position; velocity; angular_velocity; acceleration; slip_velocity; dropped;
%    vehicle_pitch; vehicle_axial_acceleration; vehicle_vertical_acceleration;
%    absolute_pipe_angle]

setup(block);

end


function setup(block)
block.NumDialogPrms = 1;
block.NumInputPorts = 1;
block.NumOutputPorts = 1;

block.InputPort(1).Dimensions = 1;
block.InputPort(1).DirectFeedthrough = true;
block.OutputPort(1).Dimensions = 10;

block.NumContStates = 3;
block.SampleTimes = [0, 0];
block.SimStateCompliance = 'DefaultSimState';

block.RegBlockMethod('InitializeConditions', @initialize_conditions);
block.RegBlockMethod('Outputs', @outputs);
block.RegBlockMethod('Derivatives', @derivatives);

end


function initialize_conditions(block)
bp = block.DialogPrm(1).Data;
block.ContStates.Data = [ ...
    bp.initial.position; ...
    bp.initial.velocity; ...
    bp.initial.omega];

end


function outputs(block)
bp = block.DialogPrm(1).Data;
state = block.ContStates.Data;
theta = block.InputPort(1).Data;
[state_dot, acceleration, slip_velocity, dropped, ...
    vehicle_pitch, vehicle_axial, vehicle_vertical, absolute_theta] = ...
    local_dynamics(state, theta, block.CurrentTime, bp);

block.OutputPort(1).Data = [ ...
    state(1); state(2); state(3); ...
    acceleration; slip_velocity; double(dropped); ...
    vehicle_pitch; vehicle_axial; vehicle_vertical; absolute_theta];

% Keep MATLAB's analyser aware that the derivative is intentionally unused here.
if isempty(state_dot)
    block.OutputPort(1).Data(:) = 0;
end

end


function derivatives(block)
bp = block.DialogPrm(1).Data;
state = block.ContStates.Data;
theta = block.InputPort(1).Data;
[state_dot, ~, ~, ~] = ...
    local_dynamics(state, theta, block.CurrentTime, bp);
block.Derivatives.Data = state_dot;

end


function [state_dot, acceleration, slip_velocity, dropped, ...
        vehicle_pitch, vehicle_axial, vehicle_vertical, absolute_theta] = ...
        local_dynamics(state, theta, time, bp)
x = state(1);
velocity = state(2);
omega = state(3);

m = bp.ball.mass;
r = bp.ball.radius;
I = bp.ball.inertia;
g = bp.gravity;
[vehicle_pitch, vehicle_axial, vehicle_vertical] = ...
    vehicle_motion_profile(time, bp);
absolute_theta = theta+vehicle_pitch;

position_limit = bp.pipe.usable_length/2 - r;
dropped = abs(x) >= position_limit;

% Once the ball has crossed an open end, latch it outside the controllable
% domain by freezing this reduced-order pipe model.
if dropped
    state_dot = zeros(3,1);
    acceleration = 0;
    slip_velocity = velocity-r*omega;
    return;
end

% Non-inertial chassis frame. Upward base acceleration increases effective
% normal load; positive axial chassis acceleration pushes the ball toward -x.
normal_acceleration = (g+vehicle_vertical)*cos(absolute_theta) ...
    +vehicle_axial*sin(absolute_theta);
normal_force = m*max(normal_acceleration, 0);
slip_velocity = velocity-r*omega;

% Stribeck transition between static and kinetic sliding friction.
mu = bp.contact.mu_kinetic ...
    + (bp.contact.mu_static-bp.contact.mu_kinetic) ...
    * exp(-(abs(slip_velocity)/bp.contact.stribeck_velocity)^2);

contact_limit = mu * normal_force;
contact_force = -contact_limit ...
    * tanh(slip_velocity/bp.contact.slip_smoothing) ...
    - bp.contact.slip_viscous*slip_velocity;

% Do not let the viscous regularisation exceed the static-friction envelope.
static_limit = bp.contact.mu_static * normal_force;
contact_force = min(max(contact_force, -static_limit), static_limit);

drag_force = -bp.drag.viscous*velocity ...
    - bp.drag.quadratic*velocity*abs(velocity);

rolling_torque = -bp.contact.rolling_resistance ...
    * normal_force*r*tanh(omega/bp.contact.omega_smoothing);

effective_axial_force = m*( ...
    (g+vehicle_vertical)*sin(absolute_theta) ...
    -vehicle_axial*cos(absolute_theta));
acceleration = (effective_axial_force+contact_force+drag_force)/m;
angular_acceleration = (-contact_force*r+rolling_torque)/I;

state_dot = [velocity; acceleration; angular_acceleration];

end
