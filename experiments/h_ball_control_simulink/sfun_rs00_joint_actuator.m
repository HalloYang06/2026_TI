function sfun_rs00_joint_actuator(block)
%SFUN_RS00_JOINT_ACTUATOR RS00 and nonlinear two-link pipe mechanism.
%
% Input:
%   [desired_pipe_angle; ball_position; ball_dropped]
%
% Output:
%   [true_pipe_angle; true_pipe_rate; motor_torque;
%    estimated_motor_current; motor_temperature; torque_saturated;
%    linkage_jacobian; linkage_valid; encoder_estimated_pipe_angle;
%    motor_angle; motor_rate]
%
% The outer controller commands pipe angle. Internally this block models:
%   - inverse/forward kinematics of the crank-coupler-pipe four-bar
%   - configuration-dependent angle ratio and equivalent inertia
%   - RS00 motion-mode Kp/Kd/torque command
%   - command and encoder quantisation plus backlash/deadband
%   - current/torque-loop lag
%   - rated/peak torque and speed-torque constraints
%   - joint friction, pipe gravity and ball-position-dependent load
%   - lumped winding/driver temperature and thermal derating

setup(block);

end


function setup(block)
block.NumDialogPrms = 1;
block.NumInputPorts = 1;
block.NumOutputPorts = 1;

block.InputPort(1).Dimensions = 3;
% Outputs depend only on continuous states. Inputs affect derivatives, so the
% RS00/ball mechanical feedback path does not create an algebraic loop.
block.InputPort(1).DirectFeedthrough = false;
block.OutputPort(1).Dimensions = 11;

% [motor angle q; motor rate q_dot; realised motor torque; temperature]
block.NumContStates = 4;
block.SampleTimes = [0, 0];
block.SimStateCompliance = 'DefaultSimState';

block.RegBlockMethod('InitializeConditions', @initialize_conditions);
block.RegBlockMethod('Outputs', @outputs);
block.RegBlockMethod('Derivatives', @derivatives);

end


function initialize_conditions(block)
bp = block.DialogPrm(1).Data;
initial_motor_angle = bp.initial.motor_angle;

if abs(bp.initial.pipe_angle) > bp.mechanism.inverse_tolerance
    [initial_motor_angle, valid] = fourbar_inverse_kinematics( ...
        bp.initial.pipe_angle, initial_motor_angle, bp);
    if ~valid
        error('Initial pipe angle is outside the valid two-link workspace.');
    end
end

[~, initial_jacobian, ~, valid] = ...
    fourbar_kinematics(initial_motor_angle, bp, false);
if ~valid
    error('Initial motor angle produces an invalid two-link assembly.');
end

initial_motor_rate = bp.initial.pipe_rate/initial_jacobian;
if isfield(bp.initial, 'motor_rate') && isfinite(bp.initial.motor_rate)
    initial_motor_rate = bp.initial.motor_rate;
end

block.ContStates.Data = [ ...
    initial_motor_angle; initial_motor_rate; 0; ...
    bp.rs00.thermal.initial];

end


function outputs(block)
bp = block.DialogPrm(1).Data;
state = block.ContStates.Data;
motor_angle = state(1);
motor_rate = state(2);
motor_torque = state(3);

[pipe_angle, jacobian, ~, linkage_valid] = ...
    fourbar_kinematics(motor_angle, bp, false);
pipe_rate = jacobian*motor_rate;

quantized_motor_angle = round( ...
    motor_angle/bp.rs00.encoder_quantization) ...
    * bp.rs00.encoder_quantization;
[encoder_pipe_angle, ~, ~, encoder_valid] = ...
    fourbar_kinematics(quantized_motor_angle, bp, false);
if ~encoder_valid
    encoder_pipe_angle = pipe_angle;
end

motor_current = motor_torque/bp.rs00.torque_constant;
speed_fraction = min(abs(motor_rate)/bp.rs00.no_load_speed, 1);
state_torque_limit = min([ ...
    bp.rs00.user_torque_limit, ...
    bp.rs00.peak_torque, ...
    bp.rs00.peak_torque*(1-speed_fraction)]);
torque_saturated = abs(motor_torque) ...
    >= 0.995*max(state_torque_limit, eps);

block.OutputPort(1).Data = [ ...
    pipe_angle; pipe_rate; motor_torque; motor_current; ...
    state(4); double(torque_saturated); jacobian; ...
    double(linkage_valid); encoder_pipe_angle; motor_angle; motor_rate];

end


function derivatives(block)
bp = block.DialogPrm(1).Data;
state = block.ContStates.Data;

motor_angle = state(1);
motor_rate = state(2);
motor_torque = state(3);
temperature = state(4);
ball_position = block.InputPort(1).Data(2);
ball_dropped = block.InputPort(1).Data(3) > 0.5;

[torque_request, ~] = local_torque_request(block, bp);
torque_derivative = (torque_request-motor_torque) ...
    /bp.rs00.current_loop_time_constant;

[pipe_angle, jacobian, ~, linkage_valid] = ...
    fourbar_kinematics(motor_angle, bp, false);
[equivalent_inertia, inertia_derivative, inertia_valid] = ...
    fourbar_equivalent_inertia(motor_angle, bp);

if ~(linkage_valid && inertia_valid)
    motor_acceleration = 0;
else
    % Gravity torque about the water-pipe pivot.
    pipe_gravity_torque = -bp.mechanism.pipe_mass*bp.gravity*( ...
        bp.mechanism.com_along_pipe*cos(pipe_angle) ...
        + bp.mechanism.com_below_pivot*sin(pipe_angle));

    % A ball away from the pipe pivot adds a configuration-dependent load.
    if ball_dropped
        ball_gravity_torque = 0;
    else
        ball_lever_arm = ball_position ...
            +bp.mechanism.ball_origin_from_pivot;
        ball_gravity_torque = ...
            -bp.ball.mass*bp.gravity*ball_lever_arm*cos(pipe_angle);
    end
    pipe_load_torque = pipe_gravity_torque+ball_gravity_torque;

    motor_friction_torque = ...
        -bp.rs00.joint_viscous_friction*motor_rate ...
        -bp.rs00.joint_coulomb_friction ...
        * tanh(motor_rate/bp.rs00.friction_smoothing);

    % Virtual work: Q_motor = (d theta/d q)*tau_pipe.
    % The 0.5*M'(q)*q_dot^2 term is required because the linkage-reflected
    % inertia varies with configuration.
    motor_acceleration = ( ...
        motor_torque+jacobian*pipe_load_torque ...
        +motor_friction_torque ...
        -0.5*inertia_derivative*motor_rate^2) ...
        /equivalent_inertia;
end

% Lumped thermal model. Replace with a fit to RS00 temperature/torque logs.
loss_power = bp.rs00.thermal.loss_per_torque_sq*motor_torque^2;
cooling_power = (temperature-bp.rs00.thermal.ambient) ...
    /bp.rs00.thermal.resistance;
temperature_derivative = (loss_power-cooling_power) ...
    /bp.rs00.thermal.capacitance;

block.Derivatives.Data = [ ...
    motor_rate; motor_acceleration; torque_derivative; ...
    temperature_derivative];

end


function [torque_request, saturated] = local_torque_request(block, bp)
state = block.ContStates.Data;
motor_angle = state(1);
motor_rate = state(2);
temperature = state(4);

desired_pipe_angle = min(max(block.InputPort(1).Data(1), ...
    -bp.actuator.max_angle), bp.actuator.max_angle);
[desired_motor_angle, inverse_valid] = fourbar_inverse_kinematics( ...
    desired_pipe_angle, motor_angle, bp);

if ~inverse_valid
    torque_request = 0;
    saturated = true;
    return;
end

quantum = bp.rs00.command_quantization;
desired_motor_angle = round(desired_motor_angle/quantum)*quantum;

position_error = desired_motor_angle-motor_angle;
if abs(position_error) <= bp.rs00.backlash_deadband
    effective_position_error = 0;
else
    effective_position_error = position_error ...
        - sign(position_error)*bp.rs00.backlash_deadband;
end

raw_torque = bp.rs00.motion_kp*effective_position_error ...
    - bp.rs00.motion_kd*motor_rate ...
    + bp.rs00.feedforward_torque;

% User limit, hardware peak limit, speed-voltage envelope and thermal derating.
torque_limit = min(bp.rs00.user_torque_limit, bp.rs00.peak_torque);
speed_fraction = min(abs(motor_rate)/bp.rs00.no_load_speed, 1);
speed_torque_limit = bp.rs00.peak_torque*(1-speed_fraction);
torque_limit = min(torque_limit, max(speed_torque_limit, 0));

if temperature <= bp.rs00.thermal.warning
    thermal_factor = 1;
elseif temperature >= bp.rs00.thermal.cutoff
    thermal_factor = 0;
else
    thermal_factor = (bp.rs00.thermal.cutoff-temperature) ...
        /(bp.rs00.thermal.cutoff-bp.rs00.thermal.warning);
end
torque_limit = torque_limit*thermal_factor;

torque_request = min(max(raw_torque, -torque_limit), torque_limit);
saturated = abs(raw_torque) > torque_limit+1e-9;

% Prevent motion farther beyond the chosen safe motor-output speed.
if abs(motor_rate) >= bp.rs00.user_speed_limit ...
        && sign(torque_request) == sign(motor_rate)
    torque_request = 0;
    saturated = true;
end

end
