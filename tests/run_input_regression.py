"""Compile the real Python binding against a fake SDK, then exercise its API.

Run with Python >= 3.10 and a C++17 compiler on Linux/macOS:
    python tests/run_input_regression.py
Build products live in a temporary directory. The production SDK is not loaded.
"""

from __future__ import annotations

import ctypes
import importlib.util
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import sysconfig
import tempfile
import threading
import time
import unittest


ROOT = Path(__file__).resolve().parents[1]


def controller(marker=1, grip=1.0):
    return {
        "pose": f"{marker},0,0,0,0,0,1",
        "trigger": 0.25, "grip": grip, "menuButton": False,
        "axisX": 0.5, "axisY": -0.5, "axisClick": False,
        "primaryButton": True, "secondaryButton": False,
    }


def frame(marker=1, grip=1.0):
    return {
        "Controller": {"left": controller(marker, grip), "right": controller(marker, grip)},
        "Head": {"pose": f"{marker},0,0,0,0,0,1"}, "Hand": {},
        "timeStampNs": marker,
    }


class InputAPIRegression(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.build_dir = tempfile.TemporaryDirectory(prefix="xrt-input-regression-")
        suffix = sysconfig.get_config_var("EXT_SUFFIX")
        extension = Path(cls.build_dir.name) / ("_core" + suffix)
        command = shlex.split(os.environ.get("CXX", "c++"))
        command += [
            "-std=c++17", "-O1", "-g", "-Wall", "-Wextra", "-fPIC",
            "-DPy_LIMITED_API=0x030A0000",
            "-I" + sysconfig.get_path("include"),
            "-I" + str(ROOT / "vendor/include"),
            str(ROOT / "bindings/py_bindings.cpp"), str(ROOT / "tests/fake_sdk.cpp"),
            "-o", str(extension),
        ]
        if sys.platform == "darwin":
            # Vendor SDK supports Linux/Windows only. This macro affects the
            # fake ABI build, not the production library/platform selection.
            command += ["-DPXREACLIENTSDK_EXPORT=", "-dynamiclib", "-undefined", "dynamic_lookup"]
        else:
            command += ["-shared", "-pthread"]
        if os.environ.get("XRT_SANITIZE"):
            command += ["-fsanitize=undefined", "-fno-sanitize-recover=all"]
        print("Build:", shlex.join(command), flush=True)
        subprocess.run(command, check=True)
        spec = importlib.util.spec_from_file_location("_core", extension)
        cls.sdk = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.sdk)
        cls.native = ctypes.CDLL(str(extension))
        cls.native.xrt_test_emit_state.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
        cls.native.xrt_test_emit_state.restype = ctypes.c_int
        cls.native.xrt_test_emit_event.argtypes = [ctypes.c_int, ctypes.c_char_p]
        cls.native.xrt_test_emit_event.restype = None
        cls.native.xrt_test_init_result.argtypes = [ctypes.c_int]
        cls.native.xrt_test_init_result.restype = None

    @classmethod
    def tearDownClass(cls):
        cls.sdk.close()
        cls.build_dir.cleanup()

    def setUp(self):
        self.sdk.close()
        self.native.xrt_test_init_result(0)
        self.sdk.set_device_id("headset-A")
        self.sdk.set_input_timeout_ms(500)
        self.sdk.init()
        self.event(1 << 2)

    def emit(self, device="headset-A", value=None, raw=None):
        if raw is None:
            raw = json.dumps({"value": json.dumps(frame() if value is None else value)})
        self.assertEqual(self.native.xrt_test_emit_state(device.encode(), raw.encode()), 0)

    def event(self, kind, device=None):
        self.native.xrt_test_emit_event(kind, None if device is None else device.encode())

    def assert_neutral(self, status):
        snapshot = self.sdk.get_input_snapshot()
        self.assertFalse(snapshot["valid"])
        self.assertEqual(snapshot["status"], status)
        self.assertEqual(snapshot["data"], {})
        self.assertEqual(self.sdk.get_left_controller_pose(), [0.0] * 7)
        self.assertEqual(self.sdk.get_right_controller_pose(), [0.0] * 7)
        self.assertEqual(self.sdk.get_left_grip(), 0.0)
        self.assertEqual(self.sdk.get_right_grip(), 0.0)
        self.assertFalse(self.sdk.get_X_button())
        self.assertEqual(self.sdk.get_time_stamp_ns(), 0)

    def test_explicit_binding_filters_interleaved_devices(self):
        for marker in range(1, 41):
            self.emit(value=frame(marker))
            self.emit("headset-B", frame(0, 0.0))
            self.assertEqual(self.sdk.get_left_controller_pose()[0], marker)
            self.assertEqual(self.sdk.get_right_grip(), 1.0)
            self.assertEqual(self.sdk.get_input_snapshot()["device_id"], "headset-A")

    def test_first_frame_binding_is_sticky_after_missing_and_reinit(self):
        self.sdk.set_device_id("")
        self.emit("headset-B", raw="{")
        self.emit("headset-A", frame(10))
        self.event(1 << 5, "headset-A")
        self.emit("headset-B", frame(20))
        self.assert_neutral("device_missing")
        self.sdk.close()
        self.sdk.init()
        self.emit("headset-B", frame(30))
        self.assert_neutral("waiting")
        self.emit(value=frame(40))
        self.assertEqual(self.sdk.get_time_stamp_ns(), 40)

    def test_unrelated_missing_and_device_find_do_not_change_binding(self):
        self.emit()
        generation = self.sdk.get_input_snapshot()["generation"]
        self.event(1 << 5, "headset-B")
        self.event(1 << 4, "headset-B")
        self.assertTrue(self.sdk.is_input_valid())
        self.assertEqual(self.sdk.get_input_snapshot()["generation"], generation)

    def test_target_missing_clears_all_modalities(self):
        value = frame()
        hand = {"scale": 1.0, "isActive": 1,
                "HandJointLocations": [{"p": "1,0,0,0,0,0,1"}] * 26}
        value["Hand"] = {"leftHand": hand, "rightHand": hand}
        value["Body"] = {"timeStampNs": 9, "joints": [
            {"p": "1,0,0,0,0,0,1", "va": "1,2,3,4,5,6",
             "wva": "6,5,4,3,2,1", "t": 9}]}
        value["Motion"] = {"timeStampNs": 8, "joints": [
            {"p": "1,0,0,0,0,0,1", "va": "1,2,3,4,5,6",
             "wva": "6,5,4,3,2,1", "sn": "tracker"}]}
        self.emit(value=value)
        self.assertTrue(self.sdk.is_body_data_available())
        self.assertEqual(self.sdk.num_motion_data_available(), 1)
        self.assertEqual(self.sdk.get_left_hand_is_active(), 1)
        self.event(1 << 5, "headset-A")
        self.assert_neutral("device_missing")
        self.assertFalse(self.sdk.is_body_data_available())
        self.assertEqual(self.sdk.get_body_joints_timestamp(), [0] * 24)
        self.assertEqual(self.sdk.get_motion_tracker_pose(), [])
        self.assertEqual(self.sdk.get_motion_tracker_serial_numbers(), [])
        self.assertEqual(self.sdk.get_left_hand_is_active(), 0)
        self.assertEqual(self.sdk.get_right_hand_tracking_state(), [[0.0] * 7] * 26)

    def test_service_disconnect_clears_until_new_target_frame(self):
        self.emit()
        generation = self.sdk.get_input_snapshot()["generation"]
        self.event(1 << 3)
        self.assert_neutral("server_disconnected")
        self.assertFalse(self.sdk.get_input_snapshot()["service_connected"])
        self.event(1 << 2)
        self.event(1 << 4, "headset-A")
        self.assertFalse(self.sdk.is_input_valid())
        self.emit("headset-B", frame(2))
        self.assertFalse(self.sdk.is_input_valid())
        self.emit(value=frame(3))
        self.assertTrue(self.sdk.is_input_valid())
        self.assertGreater(self.sdk.get_input_snapshot()["generation"], generation)

    def test_stale_is_not_disconnected_and_other_device_cannot_refresh_age(self):
        self.sdk.set_input_timeout_ms(20)
        self.emit()
        time.sleep(0.04)
        self.emit("headset-B", frame(2))
        self.assert_neutral("stale")
        snapshot = self.sdk.get_input_snapshot()
        self.assertTrue(snapshot["service_connected"])
        self.assertGreaterEqual(snapshot["age_ms"], 20)
        generation = snapshot["generation"]
        self.sdk.set_input_timeout_ms(500)
        self.assert_neutral("stale")
        self.emit()
        self.assertTrue(self.sdk.is_input_valid())
        self.assertEqual(self.sdk.get_input_snapshot()["generation"], generation)

    def test_unobserved_timeout_is_recorded_before_next_frame(self):
        self.sdk.set_input_timeout_ms(20)
        self.emit()
        generation = self.sdk.get_input_snapshot()["generation"]
        time.sleep(0.04)
        self.emit(value=frame(2))
        self.assertTrue(self.sdk.is_input_valid())
        self.assertGreater(self.sdk.get_input_snapshot()["generation"], generation)

    def test_close_ignores_late_callbacks_and_is_idempotent(self):
        self.emit()
        self.sdk.close()
        count = self.native.xrt_test_close_calls()
        self.emit(value=frame(2))
        self.assert_neutral("closed")
        self.sdk.close()
        self.assertEqual(self.native.xrt_test_close_calls(), count)
        self.sdk.init()
        self.assert_neutral("waiting")

    def test_malformed_selected_frame_never_publishes_partial_data(self):
        self.emit()
        value = frame(2)
        value["Controller"]["right"]["pose"] = "not-a-pose"
        self.emit(value=value)
        self.assert_neutral("invalid_frame")
        for pose in ("1,2,3", "nan,0,0,0,0,0,1", "1,0,0,0,0,0,1,2"):
            value = frame(3)
            value["Controller"]["left"]["pose"] = pose
            self.emit(value=value)
            self.assert_neutral("invalid_frame")
        self.emit()
        self.assertTrue(self.sdk.is_input_valid())

    def test_missing_modality_does_not_retain_old_side(self):
        self.emit()
        value = frame(2)
        del value["Controller"]["right"]
        self.emit(value=value)
        self.assertTrue(self.sdk.is_input_valid())
        self.assertEqual(self.sdk.get_right_controller_pose(), [0.0] * 7)
        self.assertEqual(self.sdk.get_right_grip(), 0.0)
        self.assertNotIn("right", self.sdk.get_input_snapshot()["data"]["Controller"])

    def test_origin_identity_and_inactive_zero_hand_poses_are_not_disconnects(self):
        value = frame(0)
        value["Hand"] = {"leftHand": {"scale": 1.0, "isActive": 0,
            "HandJointLocations": [{"p": "0,0,0,0,0,0,0"}] * 26}}
        self.emit(value=value)
        self.assertTrue(self.sdk.is_input_valid())

    def test_manual_rebind_invalidates_and_accepts_only_new_device(self):
        self.emit()
        generation = self.sdk.get_input_snapshot()["generation"]
        self.sdk.set_device_id("headset-B")
        self.assert_neutral("waiting")
        self.emit()
        self.assertFalse(self.sdk.is_input_valid())
        self.emit("headset-B", frame(2))
        self.assertEqual(self.sdk.get_time_stamp_ns(), 2)
        self.assertGreater(self.sdk.get_input_snapshot()["generation"], generation)

    def test_snapshot_is_coherent_and_returns_an_independent_copy(self):
        self.sdk.set_input_timeout_ms(5000)
        errors = []

        def producer():
            try:
                for marker in range(1, 201):
                    self.emit(value=frame(marker, marker / 200))
                    self.emit("headset-B", frame(0))
            except BaseException as error:
                errors.append(error)

        self.emit()
        thread = threading.Thread(target=producer)
        thread.start()
        for _ in range(300):
            snapshot = self.sdk.get_input_snapshot()
            data = snapshot["data"]
            marker = snapshot["timestamp_ns"]
            self.assertEqual(data["timeStampNs"], marker)
            self.assertEqual(float(data["Controller"]["left"]["pose"].split(",")[0]), marker)
            self.assertEqual(float(data["Controller"]["right"]["pose"].split(",")[0]), marker)
            snapshot["data"]["Controller"]["left"]["grip"] = -1
        thread.join(timeout=5)
        self.assertFalse(thread.is_alive())
        self.assertFalse(errors)
        self.assertGreaterEqual(self.sdk.get_input_snapshot()["data"]["Controller"]["left"]["grip"], 0)

    def test_invalid_configuration_and_duplicate_init(self):
        for timeout in (0, -1, 2**40):
            with self.assertRaises((ValueError, OverflowError)):
                self.sdk.set_input_timeout_ms(timeout)
        for device in ("x" * 32, "x\0y"):
            with self.assertRaises(RuntimeError):
                self.sdk.set_device_id(device)
        with self.assertRaises(TypeError):
            self.sdk.set_device_id(b"headset-A")
        count = self.native.xrt_test_init_calls()
        with self.assertRaises(RuntimeError):
            self.sdk.init()
        self.assertEqual(self.native.xrt_test_init_calls(), count)

    def test_init_failure_leaves_invalid_cache(self):
        self.sdk.close()
        self.native.xrt_test_init_result(-1)
        with self.assertRaises(RuntimeError):
            self.sdk.init()
        self.emit()
        self.assert_neutral("init_failed")


if __name__ == "__main__":
    unittest.main(verbosity=2)
