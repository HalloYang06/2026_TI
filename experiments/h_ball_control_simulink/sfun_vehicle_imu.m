function sfun_vehicle_imu(block)
%SFUN_VEHICLE_IMU MSPM0-sampled chassis pitch/axial-acceleration sensor.
% The current model sample time follows unique WIT source groups; a faster
% CAN mirror is represented by the held output, not by extra noisy samples.
%
% Input:
%   [true_vehicle_pitch; true_vehicle_axial_acceleration]
%
% Output:
%   [measured_pitch; measured_axial_acceleration]

setup(block);

end


function setup(block)
block.NumDialogPrms = 1;
bp = block.DialogPrm(1).Data;
block.NumInputPorts = 1;
block.NumOutputPorts = 1;
block.InputPort(1).Dimensions = 2;
block.InputPort(1).DirectFeedthrough = false;
block.OutputPort(1).Dimensions = 2;
block.SampleTimes = [bp.sample.Ts_imu, 0];
block.SimStateCompliance = 'DefaultSimState';

block.RegBlockMethod('PostPropagationSetup', @post_propagation_setup);
block.RegBlockMethod('InitializeConditions', @initialize_conditions);
block.RegBlockMethod('Outputs', @outputs);
block.RegBlockMethod('Update', @update);

end


function post_propagation_setup(block)
bp = block.DialogPrm(1).Data;
delay_samples = max(1, round(bp.sensor.imu_delay/bp.sample.Ts_imu));

block.NumDworks = 3;
local_set_dwork(block.Dwork(1), 'pitch_queue', delay_samples);
local_set_dwork(block.Dwork(2), 'acceleration_queue', delay_samples);
local_set_dwork(block.Dwork(3), 'rng_state', 1);

end


function local_set_dwork(dwork, name, width)
dwork.Name = name;
dwork.Dimensions = width;
dwork.DatatypeID = 0;
dwork.Complexity = 'Real';
dwork.UsedAsDiscState = true;

end


function initialize_conditions(block)
bp = block.DialogPrm(1).Data;
block.Dwork(1).Data(:) = 0;
block.Dwork(2).Data(:) = 0;
block.Dwork(3).Data = double(mod(bp.sensor.random_seed+9176, 2^32));

end


function outputs(block)
block.OutputPort(1).Data = [ ...
    block.Dwork(1).Data(end); ...
    block.Dwork(2).Data(end)];

end


function update(block)
bp = block.DialogPrm(1).Data;
truth = block.InputPort(1).Data;
state = block.Dwork(3).Data;

[pitch_noise, state] = local_gaussian(state);
[acceleration_noise, state] = local_gaussian(state);

pitch_measurement = truth(1)+bp.sensor.imu_pitch_bias ...
    +bp.sensor.imu_pitch_sigma*pitch_noise;
acceleration_measurement = truth(2)+bp.sensor.imu_accel_bias ...
    +bp.sensor.imu_accel_sigma*acceleration_noise;

block.Dwork(1).Data = [ ...
    pitch_measurement; block.Dwork(1).Data(1:end-1)];
block.Dwork(2).Data = [ ...
    acceleration_measurement; block.Dwork(2).Data(1:end-1)];
block.Dwork(3).Data = state;

end


function [value, state] = local_gaussian(state)
[uniform_1, state] = local_uniform(state);
[uniform_2, state] = local_uniform(state);
value = sqrt(-2*log(max(uniform_1, realmin))) ...
    * cos(2*pi*uniform_2);

end


function [value, next_state] = local_uniform(state)
next_state = mod(1664525*state+1013904223, 2^32);
value = (next_state+0.5)/2^32;

end
