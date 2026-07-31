function replay = import_edgetalk_control_log(csv_path)
%IMPORT_EDGETALK_CONTROL_LOG Load Raspberry Pi HBLG CSV for Simulink replay.
%
% replay = import_edgetalk_control_log("logs/q3_001.csv");
% The function exports hball_log_replay and individual timeseries to the
% base workspace so From Workspace blocks can use them directly.

arguments
    csv_path (1,1) string
end

table_data = readtable(csv_path, VariableNamingRule="preserve");
required = ["time_s", "ball_position_m", "estimated_position_m", ...
    "estimated_velocity_mps", "target_position_m", "pipe_target_rad", ...
    "motor_angle_rad", "motor_velocity_rad_s"];
missing = setdiff(required, string(table_data.Properties.VariableNames));
assert(isempty(missing), "Missing log columns: %s", strjoin(missing, ", "));

if any(table_data.is_q3_actual ~= 1)
    table_data = table_data(table_data.is_q3_actual == 1, :);
end
assert(height(table_data) >= 2, "Need at least two Q3 actual records");

time_s = table_data.time_s - table_data.time_s(1);
assert(all(diff(time_s) > 0), "Log time must be strictly increasing");

replay = struct();
replay.table = table_data;
replay.time_s = time_s;
replay.ball_position = timeseries(table_data.ball_position_m, time_s);
replay.estimated_position = timeseries(table_data.estimated_position_m, time_s);
replay.estimated_velocity = timeseries(table_data.estimated_velocity_mps, time_s);
replay.target_position = timeseries(table_data.target_position_m, time_s);
replay.pipe_target = timeseries(table_data.pipe_target_rad, time_s);
replay.motor_angle = timeseries(table_data.motor_angle_rad, time_s);
replay.motor_velocity = timeseries(table_data.motor_velocity_rad_s, time_s);
replay.longitudinal_accel = timeseries( ...
    table_data.longitudinal_accel_mps2, time_s);
replay.body_pitch = timeseries(table_data.body_pitch_rad, time_s);

assignin("base", "hball_log_replay", replay);
fields = fieldnames(replay);
for index = 1:numel(fields)
    value = replay.(fields{index});
    if isa(value, "timeseries")
        assignin("base", "hball_log_" + fields{index}, value);
    end
end

fprintf("Loaded %d Q3 records, duration %.3f s, median rate %.1f Hz\n", ...
    height(table_data), time_s(end), 1/median(diff(time_s)));
end
