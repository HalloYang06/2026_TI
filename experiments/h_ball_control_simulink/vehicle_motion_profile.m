function [pitch, axial_acceleration, vertical_acceleration] = ...
    vehicle_motion_profile(time, bp)
%VEHICLE_MOTION_PROFILE Deterministic chassis motion used in stress tests.

if ~isfield(bp, 'vehicle') || ~bp.vehicle.enabled
    pitch = 0;
    axial_acceleration = 0;
    vertical_acceleration = 0;
    return;
end

vehicle = bp.vehicle;
pitch = vehicle.pitch_amplitude ...
    * sin(2*pi*vehicle.pitch_frequency*time);
axial_acceleration = vehicle.axial_amplitude ...
    * sin(2*pi*vehicle.axial_frequency*time);
vertical_acceleration = vehicle.vertical_amplitude ...
    * sin(2*pi*vehicle.vertical_frequency*time);

for index = 1:size(vehicle.events, 1)
    start_time = vehicle.events(index, 1);
    duration = vehicle.events(index, 2);
    phase = (time-start_time)/max(duration, eps);
    if phase >= 0 && phase <= 1
        pulse = sin(pi*phase);
        axial_acceleration = axial_acceleration ...
            +vehicle.events(index, 3)*pulse;
        vertical_acceleration = vertical_acceleration ...
            +vehicle.events(index, 4)*pulse;
        pitch = pitch+vehicle.events(index, 5)*pulse;
    end
end

end
