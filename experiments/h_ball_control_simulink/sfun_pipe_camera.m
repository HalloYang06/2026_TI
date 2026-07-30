function sfun_pipe_camera(block)
%SFUN_PIPE_CAMERA Sampled camera with delay, noise, quantisation and dropout.
%
% Input:
%   true ball position, m
%
% Output:
%   [measured position; frame_id; valid]

setup(block);

end


function setup(block)
block.NumDialogPrms = 1;
bp = block.DialogPrm(1).Data;
block.NumInputPorts = 1;
block.NumOutputPorts = 1;
block.InputPort(1).Dimensions = 1;
block.InputPort(1).DirectFeedthrough = true;
block.OutputPort(1).Dimensions = 3;
block.SampleTimes = [bp.sample.Ts_camera, 0];
block.SimStateCompliance = 'DefaultSimState';

block.RegBlockMethod('PostPropagationSetup', @post_propagation_setup);
block.RegBlockMethod('InitializeConditions', @initialize_conditions);
block.RegBlockMethod('Outputs', @outputs);
block.RegBlockMethod('Update', @update);

end


function post_propagation_setup(block)
bp = block.DialogPrm(1).Data;
delay_frames = max(0, round(bp.sensor.vision_delay/bp.sample.Ts_camera));
queue_width = max(1, delay_frames);

block.NumDworks = 5;
local_set_dwork(block.Dwork(1), 'position_queue', queue_width);
local_set_dwork(block.Dwork(2), 'frame_queue', queue_width);
local_set_dwork(block.Dwork(3), 'valid_queue', queue_width);
local_set_dwork(block.Dwork(4), 'rng_state', 1);
local_set_dwork(block.Dwork(5), 'frame_counter', 1);

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
block.Dwork(1).Data(:) = 0;
block.Dwork(2).Data(:) = -1;
block.Dwork(3).Data(:) = 0;
block.Dwork(4).Data = double(mod(bp.sensor.random_seed, 2^32));
block.Dwork(5).Data = 0;

end


function outputs(block)
bp = block.DialogPrm(1).Data;
delay_frames = max(0, round(bp.sensor.vision_delay/bp.sample.Ts_camera));

if delay_frames == 0
    [measurement, valid] = local_measurement( ...
        block.InputPort(1).Data, block.Dwork(4).Data, bp);
    frame_id = block.Dwork(5).Data+1;
else
    measurement = block.Dwork(1).Data(end);
    frame_id = block.Dwork(2).Data(end);
    valid = block.Dwork(3).Data(end);
end

block.OutputPort(1).Data = [measurement; frame_id; valid];

end


function update(block)
bp = block.DialogPrm(1).Data;
delay_frames = max(0, round(bp.sensor.vision_delay/bp.sample.Ts_camera));
[measurement, valid, next_rng_state] = local_measurement( ...
    block.InputPort(1).Data, block.Dwork(4).Data, bp);
next_frame = block.Dwork(5).Data+1;

if delay_frames > 0
    block.Dwork(1).Data = [measurement; block.Dwork(1).Data(1:end-1)];
    block.Dwork(2).Data = [next_frame; block.Dwork(2).Data(1:end-1)];
    block.Dwork(3).Data = [valid; block.Dwork(3).Data(1:end-1)];
end

block.Dwork(4).Data = next_rng_state;
block.Dwork(5).Data = next_frame;

end


function [measurement, valid, next_state] = local_measurement(position, state, bp)
[uniform_1, state] = local_uniform(state);
[uniform_2, state] = local_uniform(state);
[uniform_3, next_state] = local_uniform(state);

gaussian = sqrt(-2*log(max(uniform_1, realmin))) * cos(2*pi*uniform_2);
measurement = position+bp.sensor.position_sigma*gaussian;

quantum = bp.sensor.position_quantization;
if quantum > 0
    measurement = round(measurement/quantum)*quantum;
end

valid = double(uniform_3 >= bp.sensor.dropout_probability);

end


function [value, next_state] = local_uniform(state)
next_state = mod(1664525*state+1013904223, 2^32);
value = (next_state+0.5)/2^32;

end
