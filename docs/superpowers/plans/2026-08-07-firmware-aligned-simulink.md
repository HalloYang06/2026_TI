# Firmware-Aligned Simulink Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the Q4/Q5/Q6 Simulink experiment use the current M33 controller, CSP limits and task profiles without changing firmware.

**Architecture:** Keep the existing `ball_pipe_nonlinear.slx` as the historical ±6° mechanism-envelope model. Add a separate firmware-aligned model builder and profile function. The new path mirrors the M33 2 ms state feedback, acceleration compensation, saturation, rate limiting and CSP constraint; it can replay Q4/Q5/Q6 HBLG records by `control_mode` and reports when V1 lacks turn inputs.

**Tech Stack:** MATLAB/Simulink S-functions, MATLAB scripts, Python `unittest` static-contract tests.

## Global Constraints

- Do not change any file under `firmware/` or `shared/`.
- Preserve the existing ±6° model and its generated historical artifacts.
- Firmware-aligned profile must use 2 ms control, Q4/Q5 `2.20/1.60/0.05`, Q6 `2.50/1.15/0.18`, longitudinal feedforward `0.55`, normal/recovery `2°/2.5°`, and CSP `Kp=120`, 5 rad/s, 2 A.
- HBLG V1 does not contain lateral acceleration or yaw rate; hardware replay must label Q6-turn conclusions unavailable when either is absent.

---

### Task 1: Add a source-traceable firmware profile

**Files:**
- Create: `experiments/h_ball_control_simulink/firmware_q456_profile.m`
- Create: `experiments/h_ball_control_simulink/tests/test_firmware_q456_profile.py`
- Modify: `experiments/h_ball_control_simulink/README.md`

**Interfaces:**
- Produces `profile = firmware_q456_profile(mission)` where `mission` is `"q4"`, `"q5"` or `"q6"`.
- `profile` fields: `control_dt_s`, `position_gain`, `velocity_gain`, `integral_gain`, `longitudinal_ff_gain`, `lateral_coupling`, `normal_limit_rad`, `recovery_limit_rad`, `normal_rate_rad_s`, `recovery_rate_rad_s`, `csp_position_kp`, `csp_speed_limit_rad_s`, `csp_current_limit_a`, `soft_start_duration_s`.

- [ ] **Step 1: Write the failing static-contract test**

```python
def test_profile_mirrors_current_firmware_constants() -> None:
    profile = PROFILE.read_text(encoding="utf-8")
    assert "profile.control_dt_s = 0.002;" in profile
    assert 'case "q4"' in profile
    assert "profile.position_gain = 2.20;" in profile
    assert "profile.velocity_gain = 1.60;" in profile
    assert "profile.integral_gain = 0.05;" in profile
    assert 'case "q6"' in profile
    assert "profile.position_gain = 2.50;" in profile
    assert "profile.velocity_gain = 1.15;" in profile
    assert "profile.integral_gain = 0.18;" in profile
```

- [ ] **Step 2: Run the test and confirm it fails because the profile does not exist**

Run: `python3 -m unittest experiments/h_ball_control_simulink/tests/test_firmware_q456_profile.py -v`

Expected: FAIL with a missing-file error.

- [ ] **Step 3: Implement the minimal profile**

```matlab
function profile = firmware_q456_profile(mission)
arguments
    mission (1,1) string {mustBeMember(mission,["q4","q5","q6"])}
end
profile.control_dt_s = 0.002;
profile.longitudinal_ff_gain = 0.55;
profile.normal_limit_rad = 2*pi/180;
profile.recovery_limit_rad = 2.5*pi/180;
profile.normal_rate_rad_s = 0.50;
profile.recovery_rate_rad_s = 0.75;
profile.csp_position_kp = 120;
profile.csp_speed_limit_rad_s = 5;
profile.csp_current_limit_a = 2;
% Select the exact Q4/Q5/Q6 gain triplet and soft-start duration here.
end
```

Add a README table that cites the M33 source files and states that this profile is manually mirrored and checked by the Python contract test.

- [ ] **Step 4: Run the test and confirm it passes**

Run: `python3 -m unittest experiments/h_ball_control_simulink/tests/test_firmware_q456_profile.py -v`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add experiments/h_ball_control_simulink/firmware_q456_profile.m \
  experiments/h_ball_control_simulink/tests/test_firmware_q456_profile.py \
  experiments/h_ball_control_simulink/README.md
git commit -m "feat(sim): add firmware Q456 profile"
```

### Task 2: Build an isolated firmware-aligned Q4/Q5/Q6 model

**Files:**
- Create: `experiments/h_ball_control_simulink/build_firmware_q456_model.m`
- Create: `experiments/h_ball_control_simulink/sfun_firmware_q456_controller.m`
- Create: `experiments/h_ball_control_simulink/run_firmware_q456_demo.m`
- Modify: `experiments/h_ball_control_simulink/sfun_rs00_joint_actuator.m`
- Create: `experiments/h_ball_control_simulink/tests/test_firmware_model_contract.py`

**Interfaces:**
- `build_firmware_q456_model(bp, mission)` returns `[model_name, model_path]` and creates `firmware_q456_<mission>.slx`.
- `sfun_firmware_q456_controller` input is `[reference; camera_x; frame_id; camera_valid; actual_pipe_angle; ball_dropped; pitch; longitudinal_accel; lateral_accel; yaw_rate]` and output is `[pipe_command; x_hat; v_hat; edge_recovery; disturbance_hat; saturated]`.
- `sfun_rs00_joint_actuator` accepts `bp.rs00.control_mode == "csp"`; current limit maps to torque limit `current_limit_a * torque_constant`.

- [ ] **Step 1: Write the failing structural test**

```python
def test_firmware_model_uses_its_own_builder_and_controller() -> None:
    builder = BUILDER.read_text(encoding="utf-8")
    controller = CONTROLLER.read_text(encoding="utf-8")
    assert "sfun_firmware_q456_controller" in builder
    assert "firmware_q456_profile" in builder
    assert "profile.control_dt_s" in controller
    assert "profile.longitudinal_ff_gain" in controller
    assert "profile.lateral_coupling" in controller
    assert "profile.normal_limit_rad" in controller
    assert "profile.recovery_limit_rad" in controller
```

- [ ] **Step 2: Run the test and confirm it fails**

Run: `python3 -m unittest experiments/h_ball_control_simulink/tests/test_firmware_model_contract.py -v`

Expected: FAIL with missing builder/controller files.

- [ ] **Step 3: Implement the discrete controller and CSP branch**

Implement prediction state `[x; v; disturbance]`, one update per new camera frame, 32 entries of 2 ms state/input history, nearest-time delayed position correction, conditional integration, the exact feedforward form `0.55*atan2(a_long - yaw^2*(0.155+x) + lateral_coupling*a_lat, 9.80665) - pitch - disturbance/((5/7)*9.80665)`, normal/recovery clamps, and 0.50/0.75 rad/s slew limits.

In `sfun_rs00_joint_actuator.m`, branch on `bp.rs00.control_mode`:

```matlab
if bp.rs00.control_mode == "csp"
    raw_torque = bp.rs00.csp_position_kp * effective_position_error;
    torque_limit = min(bp.rs00.current_limit_a * bp.rs00.torque_constant, ...
        bp.rs00.peak_torque);
else
    raw_torque = bp.rs00.motion_kp * effective_position_error ...
        - bp.rs00.motion_kd * motor_rate + bp.rs00.feedforward_torque;
end
```

The builder must use a new model name and pass 10 controller inputs. It must set `bp.sample.Ts_control` from the profile, select CSP, use the profile’s angle/rate limits, and retain the existing nonlinear plant and four-bar blocks.

- [ ] **Step 4: Run static contracts and MATLAB smoke test**

Run:

```bash
python3 -m unittest \
  experiments/h_ball_control_simulink/tests/test_firmware_q456_profile.py \
  experiments/h_ball_control_simulink/tests/test_firmware_model_contract.py -v
```

Then in MATLAB:

```matlab
cd experiments/h_ball_control_simulink
run_firmware_q456_demo("q5")
```

Expected: model builds, runs at 2 ms, reports its active profile, and never exceeds ±2° normal/±2.5° recovery limits.

- [ ] **Step 5: Commit**

```bash
git add experiments/h_ball_control_simulink
git commit -m "feat(sim): mirror Q456 firmware controller and CSP"
```

### Task 3: Make HBLG replay mission-aware and honest about missing turn data

**Files:**
- Modify: `experiments/h_ball_control_simulink/import_edgetalk_control_log.m`
- Modify: `experiments/h_ball_control_simulink/analyze_edgetalk_q6_log.m`
- Create: `experiments/h_ball_control_simulink/analyze_edgetalk_q45_log.m`
- Modify: `experiments/h_ball_control_simulink/README.md`
- Create: `experiments/h_ball_control_simulink/tests/test_hblg_replay_contract.py`

**Interfaces:**
- `import_edgetalk_control_log(csv_path, mission)` accepts mission `"auto"`, `"q3"`, `"q4"`, `"q5"`, `"q6"`, or `"all"`.
- It maps `control_mode` `0x0101/0x0102/0x0103` to Q3/Q4-Q5/Q6 and exposes `replay.turn_replay_valid`.

- [ ] **Step 1: Write the failing importer contract test**

```python
def test_importer_filters_by_control_mode_not_q3_status_bit() -> None:
    source = IMPORTER.read_text(encoding="utf-8")
    assert 'mission (1,1) string = "auto"' in source
    assert "uint16(table_data.control_mode)" in source
    assert "turn_replay_valid" in source
    assert "is_q3_actual == 1" not in source
```

- [ ] **Step 2: Run the test and confirm it fails**

Run: `python3 -m unittest experiments/h_ball_control_simulink/tests/test_hblg_replay_contract.py -v`

Expected: FAIL because the importer has no mission argument and filters through `is_q3_actual`.

- [ ] **Step 3: Implement mission filtering and replay metrics**

Use the `control_mode` lower byte: `1` for Q3, `2` for Q4/Q5 centre hold, `3` for Q6. `"auto"` selects the unique active mode and errors on a mixed-mode capture. Preserve `target_position_m` from the current V1 context slot for active controller records. Set `turn_replay_valid=false` because V1 has no lateral acceleration/yaw columns; `analyze_edgetalk_q6_log` must print that Q6 results describe straight/aggregate tracking only.

- [ ] **Step 4: Run static contracts and a MATLAB CSV smoke test**

Run:

```bash
python3 -m unittest experiments/h_ball_control_simulink/tests/test_hblg_replay_contract.py -v
```

Then in MATLAB with a single-mode capture:

```matlab
replay = import_edgetalk_control_log("q6.csv", "q6");
metrics = analyze_edgetalk_q6_log("q6.csv");
assert(~replay.turn_replay_valid)
```

Expected: Q6 records are retained by mode, target and error metrics are finite, and the report states the turn-data limitation.

- [ ] **Step 5: Commit**

```bash
git add experiments/h_ball_control_simulink
git commit -m "fix(sim): make HBLG replay mission-aware"
```

### Task 4: Record calibration inputs without claiming them measured

**Files:**
- Create: `experiments/h_ball_control_simulink/fit_hardware_replay.m`
- Modify: `experiments/h_ball_control_simulink/README.md`
- Create: `experiments/h_ball_control_simulink/tests/test_hardware_fit_contract.py`

**Interfaces:**
- `fit = fit_hardware_replay(csv_path, mission)` returns `camera_delay_s`, `csp_delay_s`, `rolling_resistance`, `position_rms_mm`, `position_peak_mm`, `accepted`.
- It searches bounded candidates and does not change firmware or overwrite `ball_pipe_defaults.m` automatically.

- [ ] **Step 1: Write the failing fit-contract test**

```python
def test_fitter_never_writes_firmware_or_default_parameters() -> None:
    source = FITTER.read_text(encoding="utf-8")
    assert "camera_delay_candidates_s" in source
    assert "csp_delay_candidates_s" in source
    assert "rolling_resistance_candidates" in source
    assert "writematrix" not in source
    assert "fopen" not in source
```

- [ ] **Step 2: Run the test and confirm it fails**

Run: `python3 -m unittest experiments/h_ball_control_simulink/tests/test_hardware_fit_contract.py -v`

Expected: FAIL because the fitter file does not exist.

- [ ] **Step 3: Implement bounded offline fit**

Use candidates `camera_delay_s = 0.010:0.005:0.080`, `csp_delay_s = 0.002:0.002:0.030`, and `rolling_resistance = 0.001:0.001:0.012`. For each candidate, replay the logged pipe command through the nonlinear plant, compute ball-position RMS/peak only over matching timestamps, and return the minimum-RMS candidate. Require at least 2 s of monotonic time and reject mixed mission records.

- [ ] **Step 4: Run static contract and MATLAB smoke test**

Run: `python3 -m unittest experiments/h_ball_control_simulink/tests/test_hardware_fit_contract.py -v`

Then in MATLAB:

```matlab
fit = fit_hardware_replay("q5.csv", "q5");
assert(isfinite(fit.position_rms_mm))
```

Expected: fitter reports candidate values and errors without writing any source or firmware file.

- [ ] **Step 5: Commit**

```bash
git add experiments/h_ball_control_simulink
git commit -m "feat(sim): fit Q456 model from hardware replay"
```
