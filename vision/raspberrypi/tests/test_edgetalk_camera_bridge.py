from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[3]
VISION_DIR = ROOT / "vision" / "raspberrypi"
MODULE_PATH = VISION_DIR / "edgetalk_camera_bridge.py"
CAMERA_SOURCE_PATH = VISION_DIR / "ball_camera" / "ball_vision_sender.cpp"
CAMERA_START_PATH = VISION_DIR / "ball_camera" / "start_ball_camera.sh"
CAMERA_USER_SERVICE_PATH = (
    VISION_DIR / "systemd" / "hball-edgetalk-camera-user.service"
)


def load_bridge_module():
    sys.path.insert(0, str(VISION_DIR))
    spec = importlib.util.spec_from_file_location("edgetalk_camera_bridge", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_real_camera_bridge_does_not_claim_unmeasured_quality_flags():
    bridge = load_bridge_module()
    measurement = bridge.build_measurement({
        "found": True,
        "sequence": 7,
        "capture_time_us": 123456,
        "processing_time_us": 1200,
        "position_cm": 2.5,
    })

    assert measurement.flags & bridge.FLAG_DETECTED
    assert measurement.flags & bridge.FLAG_POSITION_VALID
    assert measurement.flags & (1 << 4) == 0  # EXPOSURE_STABLE is not measured.
    assert measurement.flags & (1 << 5) == 0  # CONTOUR_ROUND is not reported.
    assert measurement.sequence == 7
    assert measurement.ball_position_m == 0.025


def test_missing_ball_clears_position_valid_and_confidence():
    bridge = load_bridge_module()
    measurement = bridge.build_measurement({
        "found": False,
        "sequence": 8,
        "capture_time_us": 124000,
    })

    assert measurement.flags == 0
    assert measurement.ball_position_m == 0.0
    assert measurement.confidence == 0.0


def test_real_camera_defaults_match_the_frozen_100_hz_target():
    source = CAMERA_SOURCE_PATH.read_text(encoding="utf-8")
    start_script = CAMERA_START_PATH.read_text(encoding="utf-8")

    assert "int fps = 100;" in source
    assert "--fps 100" in start_script
    assert "--fps 120" not in start_script


def test_real_camera_user_service_is_restartable_and_identity_neutral():
    service = CAMERA_USER_SERVICE_PATH.read_text(encoding="utf-8")

    assert "Restart=always" in service
    assert "EnvironmentFile=%h/.config/hball/edgetalk-camera.env" in service
    assert "${HBALL_CAMERA_BRIDGE}" in service
    assert "${HBALL_CAMERA_SOURCE_URL}" in service
    assert "%t/hball-edgetalk-usb.lock" in service
    assert "User=" not in service
    for forbidden in ("halloyang", "192.168.", "ttyACM", "password"):
        assert forbidden not in service
