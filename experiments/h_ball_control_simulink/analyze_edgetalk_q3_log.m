function metrics = analyze_edgetalk_q3_log(csv_path)
%ANALYZE_EDGETALK_Q3_LOG Calculate Q3 timing/error metrics and plot replay.

replay = import_edgetalk_control_log(csv_path);
t = replay.time_s;
target = replay.target_position.Data;
position = replay.ball_position.Data;
error_mm = 1000*(target-position);

metrics.duration_s = t(end);
metrics.rms_error_mm = sqrt(mean(error_mm.^2, "omitnan"));
metrics.max_abs_error_mm = max(abs(error_mm), [], "omitnan");
metrics.final_error_mm = error_mm(end);
metrics.pass_time = metrics.duration_s <= 5.0;
metrics.pass_final_error = abs(metrics.final_error_mm) <= 10.0;
metrics.pass = metrics.pass_time && metrics.pass_final_error;

figure(Name="EdgeTalk Q3 hardware replay");
tiledlayout(3,1);
nexttile;
plot(t, 1000*target, "--", t, 1000*position, LineWidth=1.2);
ylabel("position (mm)"); grid on; legend("target", "measured");
nexttile;
plot(t, error_mm, LineWidth=1.2); yline(10, ":"); yline(-10, ":");
ylabel("error (mm)"); grid on;
nexttile;
plot(t, replay.pipe_target.Data*180/pi, LineWidth=1.2);
ylabel("pipe (deg)"); xlabel("time (s)"); grid on;

fprintf("Q3 duration=%.3f s final_error=%.2f mm max_error=%.2f mm pass=%d\n", ...
    metrics.duration_s, metrics.final_error_mm, ...
    metrics.max_abs_error_mm, metrics.pass);
end
