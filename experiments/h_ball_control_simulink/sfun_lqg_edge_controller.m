function sfun_lqg_edge_controller(block)
%SFUN_LQG_EDGE_CONTROLLER KF/linear-ESO + LQI with edge recovery.
%
% Input:
%   [reference; camera_position; frame_id; camera_valid;
%    actual_pipe_angle; ball_dropped; imu_pitch; imu_axial_acceleration]
%
% Output:
%   [angle_command; estimated_position; estimated_velocity; edge_recovery;
%    estimated_lumped_acceleration_disturbance]

setup(block);

end


function setup(block)
block.NumDialogPrms = 1;
bp = block.DialogPrm(1).Data;
block.NumInputPorts = 1;
block.NumOutputPorts = 1;
block.InputPort(1).Dimensions = 8;
block.InputPort(1).DirectFeedthrough = true;
block.OutputPort(1).Dimensions = 5;
block.SampleTimes = [bp.sample.Ts_control, 0];
block.SimStateCompliance = 'DefaultSimState';

block.RegBlockMethod('PostPropagationSetup', @post_propagation_setup);
block.RegBlockMethod('InitializeConditions', @initialize_conditions);
block.RegBlockMethod('Outputs', @outputs);
block.RegBlockMethod('Update', @update);

end


function post_propagation_setup(block)
block.NumDworks = 4;
local_set_dwork(block.Dwork(1), 'state_estimate', 3);
local_set_dwork(block.Dwork(2), 'covariance', 9);
local_set_dwork(block.Dwork(3), 'integral_error', 1);
local_set_dwork(block.Dwork(4), 'last_frame_id', 1);

end


function local_set_dwork(dwork, name, width)
dwork.Name = name;
dwork.Dimensions = width;
dwork.DatatypeID = 0; % double
dwork.Complexity = 'Real';
dwork.UsedAsDiscState = true;

end


function initialize_conditions(block)
bp = block.DialogPrm(1).Data;
block.Dwork(1).Data = [0; 0; 0];
block.Dwork(2).Data = reshape(diag([ ...
    bp.kalman.initial_position_sigma^2, ...
    bp.kalman.initial_velocity_sigma^2, ...
    bp.kalman.initial_disturbance_sigma^2]), 9, 1);
block.Dwork(3).Data = 0;
block.Dwork(4).Data = -1;

end


function outputs(block)
bp = block.DialogPrm(1).Data;
input = block.InputPort(1).Data;
reference = input(1);
dropped = input(6) > 0.5;
imu_pitch = input(7);
imu_axial_acceleration = input(8);

state_estimate = block.Dwork(1).Data;
position_estimate = state_estimate(1);
velocity_estimate = state_estimate(2);
disturbance_estimate = state_estimate(3);
integral_error = block.Dwork(3).Data;

position_error = position_estimate-reference;
vehicle_feedforward = 0;
if bp.controller.vehicle_feedforward_enabled
    vehicle_feedforward = ...
        imu_axial_acceleration/bp.gravity-imu_pitch;
end

disturbance_compensation = 0;
if bp.controller.disturbance_observer_enabled
    disturbance_compensation = ...
        -disturbance_estimate/bp.nominal.acceleration_gain;
    disturbance_compensation = min(max(disturbance_compensation, ...
        -bp.controller.disturbance_compensation_limit), ...
        bp.controller.disturbance_compensation_limit);
end
total_feedforward = vehicle_feedforward+disturbance_compensation;

normal_command = total_feedforward-bp.controller.K * [ ...
    position_error; velocity_estimate; integral_error];

outward_velocity = sign(position_estimate)*velocity_estimate;
stopping_distance = max(outward_velocity, 0)^2 ...
    /(2*max(bp.controller.recovery_accel, eps));
remaining_distance = bp.controller.position_limit-abs(position_estimate);
edge_recovery = remaining_distance ...
    <= bp.controller.edge_margin+stopping_distance;

if dropped
    angle_command = 0;
    edge_recovery = true;
elseif edge_recovery
    if abs(position_estimate) > 1e-6
        angle_command = total_feedforward-sign(position_estimate) ...
            * bp.controller.recovery_angle;
    else
        angle_command = normal_command;
    end
else
    angle_command = normal_command;
end

angle_command = min(max(angle_command, ...
    -bp.actuator.max_angle), bp.actuator.max_angle);
block.OutputPort(1).Data = [ ...
    angle_command; position_estimate; velocity_estimate; ...
    double(edge_recovery); disturbance_estimate];

end


function update(block)
bp = block.DialogPrm(1).Data;
input = block.InputPort(1).Data;
reference = input(1);
measurement = input(2);
frame_id = input(3);
measurement_valid = input(4) > 0.5;
actual_angle = input(5);
dropped = input(6) > 0.5;
imu_pitch = input(7);
imu_axial_acceleration = input(8);

Ts = bp.sample.Ts_control;
b = bp.nominal.acceleration_gain;
A = [ ...
    1, Ts, 0.5*Ts^2; ...
    0, 1, Ts; ...
    0, 0, 1];
B = [0.5*b*Ts^2; b*Ts; 0];

acceleration_variance = bp.kalman.acceleration_sigma^2;
acceleration_noise_input = [0.5*Ts^2; Ts; 0];
Q = acceleration_variance ...
    *(acceleration_noise_input*acceleration_noise_input');
Q(3,3) = Q(3,3) ...
    +(bp.kalman.disturbance_random_walk_sigma*Ts)^2;
R = bp.sensor.position_sigma^2 ...
    + bp.sensor.position_quantization^2/12;

state_estimate = block.Dwork(1).Data;
covariance = reshape(block.Dwork(2).Data, 3, 3);

effective_angle = actual_angle+imu_pitch ...
    -imu_axial_acceleration/bp.gravity;
state_prediction = A*state_estimate+B*effective_angle;
covariance_prediction = A*covariance*A'+Q;

is_new_frame = frame_id >= 0 && frame_id ~= block.Dwork(4).Data;
if is_new_frame
    block.Dwork(4).Data = frame_id;
end

if is_new_frame && measurement_valid
    % The Raspberry-Pi position belongs to an older capture time. Relate
    % that delayed position directly to the CURRENT augmented state instead
    % of treating it as a zero-latency measurement:
    % x(t-tau) ~= x(t)-tau*v(t)+0.5*tau^2*(b*u+d).
    measurement_delay = round( ...
        bp.sensor.vision_delay/bp.sample.Ts_camera) ...
        * bp.sample.Ts_camera;
    H_delayed = [1, -measurement_delay, 0.5*measurement_delay^2];
    known_input_offset = 0.5*b*effective_angle*measurement_delay^2;
    innovation = measurement ...
        -(H_delayed*state_prediction+known_input_offset);
    innovation_variance = ...
        H_delayed*covariance_prediction*H_delayed'+R;
    gate = bp.controller.innovation_gate_sigma ...
        * sqrt(max(innovation_variance, eps));
    if abs(innovation) <= gate
        kalman_gain = ...
            covariance_prediction*H_delayed'/innovation_variance;
        state_prediction = state_prediction+kalman_gain*innovation;
        covariance_prediction = ...
            (eye(3)-kalman_gain*H_delayed)*covariance_prediction;
    end
end

if ~bp.controller.disturbance_observer_enabled
    state_prediction(3) = 0;
    covariance_prediction(3,:) = 0;
    covariance_prediction(:,3) = 0;
    covariance_prediction(3,3) = ...
        bp.kalman.initial_disturbance_sigma^2;
end

% Symmetrise to suppress numerical covariance drift.
covariance_prediction = ...
    0.5*(covariance_prediction+covariance_prediction');
block.Dwork(1).Data = state_prediction;
block.Dwork(2).Data = reshape(covariance_prediction, 9, 1);

if dropped
    block.Dwork(3).Data = 0;
else
    current_integral = block.Dwork(3).Data;
    candidate_integral = current_integral ...
        + Ts*(state_prediction(1)-reference);
    candidate_integral = min(max(candidate_integral, ...
        -bp.controller.integral_limit), bp.controller.integral_limit);

    vehicle_feedforward = 0;
    if bp.controller.vehicle_feedforward_enabled
        vehicle_feedforward = ...
            imu_axial_acceleration/bp.gravity-imu_pitch;
    end
    disturbance_compensation = 0;
    if bp.controller.disturbance_observer_enabled
        disturbance_compensation = ...
            -state_prediction(3)/bp.nominal.acceleration_gain;
        disturbance_compensation = min(max(disturbance_compensation, ...
            -bp.controller.disturbance_compensation_limit), ...
            bp.controller.disturbance_compensation_limit);
    end
    total_feedforward = vehicle_feedforward+disturbance_compensation;

    candidate_command = total_feedforward-bp.controller.K * [ ...
        state_prediction(1)-reference; ...
        state_prediction(2); candidate_integral];

    % Conditional integration also accounts for a rate-limited mechanism.
    % If the actuator is lagging and the integral increment would demand
    % still more motion in the same direction, freeze the integrator.
    current_command = total_feedforward-bp.controller.K * [ ...
        state_prediction(1)-reference; ...
        state_prediction(2); current_integral];
    current_command = min(max(current_command, ...
        -bp.actuator.max_angle), bp.actuator.max_angle);
    command_increment_from_integral = ...
        -bp.controller.K(3)*(candidate_integral-current_integral);
    actuator_lag = current_command-actual_angle;
    tracking_allows_integration = ...
        abs(actuator_lag) <= bp.controller.actuator_tracking_tolerance ...
        || actuator_lag*command_increment_from_integral <= 0;

    if abs(candidate_command) <= bp.actuator.max_angle ...
            && tracking_allows_integration
        block.Dwork(3).Data = candidate_integral;
    end
end

end
