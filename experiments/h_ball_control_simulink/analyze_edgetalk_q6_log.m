function metrics = analyze_edgetalk_q6_log(csv_path)
%ANALYZE_EDGETALK_Q6_LOG Analyze one Q6 hardware run for Simulink tuning.

replay = import_edgetalk_control_log(csv_path);
t = replay.time_s;
target = replay.target_position.Data;
position = replay.ball_position.Data;
error_m = target - position;
table_data = replay.table;

steady_start = max(t(1), t(end) - min(3.0, 0.25*t(end)));
steady = t >= steady_start;
initial_position = median(position(t <= min(t(end), t(1) + 0.25)), "omitnan");
direction = sign(median(target, "omitnan") - initial_position);
if direction == 0
    direction = 1;
end

metrics.duration_s = t(end);
metrics.sample_rate_hz = 1/median(diff(t));
metrics.target_cm = 100*median(target, "omitnan");
metrics.final_position_cm = 100*position(end);
metrics.steady_position_cm = 100*mean(position(steady), "omitnan");
metrics.steady_error_cm = 100*mean(error_m(steady), "omitnan");
metrics.steady_std_cm = 100*std(position(steady), "omitnan");
metrics.max_abs_error_cm = 100*max(abs(error_m), [], "omitnan");
metrics.overshoot_cm = 100*max(direction*(position-target), [], "omitnan");
metrics.max_pipe_deg = max(abs(replay.pipe_target.Data))*180/pi;

within_1cm = abs(error_m) <= 0.01;
metrics.within_1cm_fraction = mean(within_1cm);
metrics.settling_time_1cm_s = NaN;
for index = 1:numel(t)
    if all(within_1cm(index:end))
        metrics.settling_time_1cm_s = t(index);
        break;
    end
end

metrics.imu_valid_fraction = NaN;
metrics.imu_dropout_events = NaN;
if ismember("sensor_valid_flags", string(table_data.Properties.VariableNames))
    imu_valid = bitget(uint32(table_data.sensor_valid_flags), 2) ~= 0;
    metrics.imu_valid_fraction = mean(imu_valid);
    metrics.imu_dropout_events = sum(diff([true; imu_valid]) == -1);
end

figure(Name="EdgeTalk Q6 hardware replay");
tiledlayout(4,1);
nexttile;
plot(t, 100*target, "--", t, 100*position, LineWidth=1.2);
ylabel("position (cm)"); grid on; legend("target", "measured");
nexttile;
plot(t, 100*error_m, LineWidth=1.2); yline(1, ":"); yline(-1, ":");
ylabel("error (cm)"); grid on;
nexttile;
plot(t, replay.pipe_target.Data*180/pi, LineWidth=1.2);
ylabel("pipe (deg)"); grid on;
nexttile;
if ismember("sensor_valid_flags", string(table_data.Properties.VariableNames))
    stairs(t, imu_valid, LineWidth=1.2); ylim([-0.1 1.1]);
end
ylabel("IMU valid"); xlabel("time (s)"); grid on;

fprintf(["Q6 target=%.2f cm steady=%.2f cm error=%.2f cm " ...
    "overshoot=%.2f cm IMU=%.1f%% drops=%g\n"], ...
    metrics.target_cm, metrics.steady_position_cm, metrics.steady_error_cm, ...
    metrics.overshoot_cm, 100*metrics.imu_valid_fraction, ...
    metrics.imu_dropout_events);
end
