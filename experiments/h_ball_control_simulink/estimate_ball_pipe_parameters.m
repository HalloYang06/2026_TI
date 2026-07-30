function estimate = estimate_ball_pipe_parameters( ...
        angle_deg, travel_m, travel_time_s, motion_mode)
%ESTIMATE_BALL_PIPE_PARAMETERS Estimate resistance/friction from an incline test.
%
% Examples:
%   % A visibly rolling ball travelled 0.40 m in 1.10 s at 3 degrees:
%   e = estimate_ball_pipe_parameters(3, 0.40, 1.10, 'rolling')
%
%   % A visibly sliding ball travelled the same distance/time:
%   e = estimate_ball_pipe_parameters(3, 0.40, 1.10, 'sliding')
%
% Assumptions:
%   - released from rest
%   - constant pipe angle
%   - resistance approximately constant over the measured run
%   - aerodynamic/viscous drag is small during this identification test

arguments
    angle_deg (1,1) double
    travel_m (1,1) double {mustBePositive}
    travel_time_s (1,1) double {mustBePositive}
    motion_mode (1,:) char {mustBeMember(motion_mode, {'rolling','sliding'})}
end

g = 9.80665;
theta = angle_deg*pi/180;
acceleration = 2*travel_m/travel_time_s^2;

estimate.angle_deg = angle_deg;
estimate.travel_m = travel_m;
estimate.travel_time_s = travel_time_s;
estimate.acceleration_mps2 = acceleration;
estimate.motion_mode = motion_mode;

switch motion_mode
    case 'rolling'
        % a = (5/7)g(sin(theta)-Crr*cos(theta))
        estimate.rolling_resistance = ...
            (sin(theta)-(7/5)*acceleration/g)/cos(theta);
        estimate.mu_kinetic = NaN;
    case 'sliding'
        % a = g(sin(theta)-mu_k*cos(theta))
        estimate.mu_kinetic = ...
            (sin(theta)-acceleration/g)/cos(theta);
        estimate.rolling_resistance = NaN;
end

fprintf('Measured acceleration: %.5f m/s^2\n', acceleration);
if strcmp(motion_mode, 'rolling')
    fprintf('Estimated equivalent rolling resistance Crr: %.5f\n', ...
        estimate.rolling_resistance);
else
    fprintf('Estimated kinetic sliding coefficient mu_k: %.5f\n', ...
        estimate.mu_kinetic);
end
fprintf(['Check the ball with a visible dot: rolling requires ' ...
    'approximately v = radius*omega.\n']);

end
