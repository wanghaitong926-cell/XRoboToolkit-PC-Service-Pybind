# XRoboToolkit-PC-Service-Pybind

This project provides a python interface to extract XR state using XRoboToolkit-PC-Service sdk.

## Requirements

- Linux (x86_64 or aarch64, e.g. NVIDIA Orin) or Windows (x64), Python >= 3.10
- A running [XRoboToolkit PC Service](https://github.com/XR-Robotics/XRoboToolkit-PC-Service) with a connected PICO headset (at runtime)

The prebuilt `PXREARobotSDK` binaries are vendored in `vendor/`, so no manual
SDK download or build is needed.

## Installation

### From PyPI

```bash
uv add xrobotoolkit-sdk
# or
pip install xrobotoolkit-sdk
```

### From source

If you do not have [uv](https://docs.astral.sh/uv/) installed, run:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Then (requires a C++17 compiler; CMake >= 3.26 is downloaded automatically if
not installed):

```bash
git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind.git
cd XRoboToolkit-PC-Service-Pybind
uv sync
```

`uv sync` creates `.venv/` with any available Python >= 3.10 (pick one with
e.g. `uv sync --python 3.12`), builds the extension, and installs the package
into it. Run scripts with e.g. `uv run examples/example.py`.

To build distributable artifacts locally: `uv build`. The extension is built
against the CPython stable ABI, so the wheel (e.g.
`xrobotoolkit_sdk-1.1.0-cp310-abi3-win_amd64.whl`) installs on any CPython
>= 3.10 for the same OS and architecture, whichever Python version built it.
Wheels for Linux/Windows are also built in CI via cibuildwheel, see
`.github/workflows/wheels.yml`.

## Using the Python Bindings

### Selecting a device and checking input freshness

```python
import xrobotoolkit_sdk as xrt

xrt.set_device_id("YOUR_HEADSET_DEVICE_ID")  # SDK devID, not an IP address
xrt.set_input_timeout_ms(500)
xrt.init()
try:
    snapshot = xrt.get_input_snapshot()
    if snapshot["valid"]:
        # All fields in data come from the same device and complete message.
        controllers = snapshot["data"].get("Controller", {})
        left = controllers.get("left")
        right = controllers.get("right")
        if left is not None and right is not None:
            left_pose = [float(v) for v in left["pose"].split(",")]
            right_pose = [float(v) for v in right["pose"].split(",")]
            left_grip = left["grip"]
            right_grip = right["grip"]
            # Check tracking quality and application takeover state before use.
finally:
    xrt.close()
```

`set_device_id()` selects the only device allowed to publish input. Device IDs
are the SDK's `devID` values (at most 31 UTF-8 bytes), as reported by the PC
Service. For compatibility, an unconfigured `init()` pins the first successfully
parsed frame's nonempty device ID. This prevents mixing but cannot determine
which headset an operator intended to use: **set an explicit ID when multiple
devices are present**.

The binding survives missing-device events, stale input, service reconnects and
`close()` / `init()`. It never automatically switches to another headset.
Calling `set_device_id("OTHER_ID")` explicitly switches and invalidates input;
calling `set_device_id("")` explicitly permits first-frame binding again.

`get_input_snapshot()` returns:

- `device_id`: selected SDK identity.
- `valid`: a successfully parsed, unexpired message is available.
- `status`: `uninitialized`, `waiting`, `active`, `stale`, `device_missing`,
  `server_disconnected`, `invalid_frame`, `init_failed` or `closed`.
- `generation`: increases on binding or invalidation. Applications can detect a
  discontinuity even if fresh data arrived before the next read.
- `service_connected`: last SDK service-connect/disconnect event state;
  it is not a headset tracking-quality indicator.
- `age_ms`: elapsed **local steady-clock** time since the last accepted frame,
  or `None` when no receipt is retained. Unrelated-device frames do not refresh it.
- `timeout_ms`: positive receipt deadline, default 500 ms.
- `timestamp_ns`: selected message's source timestamp, or zero when invalid.
- `data`: decoded `value` object from that one message; pose fields retain the
  wire format (comma-separated strings). An invalid snapshot contains `{}`.

`is_input_valid()` provides the same validity check without copying a snapshot.
Receipt expiry is checked on every read and before publishing the next accepted
frame; it requires no watchdog thread.

**Disconnect detection is separate from freshness.** `device_missing` is set
only for a matching `PXREADeviceMissing` event; `server_disconnected` is set for
`PXREAServerDisconnect`. In the current upstream PC Service,
`TcpServerWorker::onCheckConnection()` reports a missing device after more than
20 seconds without a device heartbeat. That is a service-reported timeout, not
proof that a headset has physically powered off. The shorter configurable
receipt deadline marks input `stale` without declaring a transport disconnect.
It uses local receipt time, not the headset's clock.

Existing getter names, no-argument signatures and return shapes remain. Their
behavior intentionally changes: unrelated devices cannot overwrite them;
missing fields start at neutral values each frame; and invalidated/expired
poses, buttons, grips, timestamps and tracking caches return neutral defaults
instead of retaining the last values. Separate legacy getter calls can still
span two accepted frames, so use `get_input_snapshot()` for a coherent read.
Calling `init()` twice without `close()` now raises; `close()` is idempotent.

Validity means a parsed message is fresh, **not** that all controllers are
tracked, that source timestamps are progressing, or that robot motion may
resume. A neutral pose is not a valid control target. Consumers must inhibit
control when input is invalid, inspect modality presence/tracking quality, and
require their own release/re-arm/takeover procedure after a generation change.
The SDK does not move a robot or establish a new takeover reference.

### Offline API regression

On Linux or macOS with Python >= 3.10 and a C++17 compiler:

```bash
python tests/run_input_regression.py
```

This compiles the real binding with `Py_LIMITED_API=0x030A0000` against a
test-only SDK ABI replacement, injects interleaved device frames and lifecycle
events, and exercises the Python APIs. It opens no SDK socket or hardware
connection and builds only in a temporary directory. macOS is supported for
this fake-SDK regression, not for production SDK wheels.

**1. Get Controller and Headset Poses**

```python
import xrobotoolkit_sdk as xrt

xrt.init()

left_pose = xrt.get_left_controller_pose()
right_pose = xrt.get_right_controller_pose()
headset_pose = xrt.get_headset_pose()

print(f"Left Controller Pose: {left_pose}")
print(f"Right Controller Pose: {right_pose}")
print(f"Headset Pose: {headset_pose}")

xrt.close()
```

**2. Get Controller Inputs (Triggers, Grips, Buttons, Axes)**

```python
import xrobotoolkit_sdk as xrt

xrt.init()

# Triggers and Grips
left_trigger = xrt.get_left_trigger()
right_grip = xrt.get_right_grip()
print(f"Left Trigger: {left_trigger}, Right Grip: {right_grip}")

# Buttons
a_button_pressed = xrt.get_A_button()
x_button_pressed = xrt.get_X_button()
print(f"A Button Pressed: {a_button_pressed}, X Button Pressed: {x_button_pressed}")

# Axes
left_axis = xrt.get_left_axis()
right_axis_click = xrt.get_right_axis_click()
print(f"Left Axis: {left_axis}, Right Axis Clicked: {right_axis_click}")

# Timestamp
timestamp = xrt.get_time_stamp_ns()
print(f"Current Timestamp (ns): {timestamp}")

xrt.close()
```

**3. Get hand tracking state**
```python
import xrobotoolkit_sdk as xrt

xrt.init()

# Left Hand State
left_hand_tracking_state = xrt.get_left_hand_tracking_state()
print(f"Left Hand State: {left_hand_tracking_state}")

# Left Hand isActive
left_hand_is_active = xrt.get_left_hand_is_active()
print(f"Left Hand isActive: {left_hand_is_active}")

# Right Hand State
right_hand_tracking_state = xrt.get_right_hand_tracking_state()
print(f"Right Hand State: {right_hand_tracking_state}")

# Right Hand isActive
right_hand_is_active = xrt.get_right_hand_is_active()
print(f"Right Hand isActive: {right_hand_is_active}")

xrt.close()
```

**4. Get whole body motion tracking （please refer to this example when check Full Body tracking mode in UNITY app）**
```python
import xrobotoolkit_sdk as xrt

xrt.init()

# Check if body tracking data is available
if xrt.is_body_data_available():
    # Get body joint poses (24 joints, 7 values each: x,y,z,qx,qy,qz,qw)
    body_poses = xrt.get_body_joints_pose()
    print(f"Body joints pose data: {body_poses}")
    
    # Get body joint velocities (24 joints, 6 values each: vx,vy,vz,wx,wy,wz)
    body_velocities = xrt.get_body_joints_velocity()
    print(f"Body joints velocity data: {body_velocities}")
    
    # Get body joint accelerations (24 joints, 6 values each: ax,ay,az,wax,way,waz)
    body_accelerations = xrt.get_body_joints_acceleration()
    print(f"Body joints acceleration data: {body_accelerations}")
    
    # Get IMU timestamps for each joint
    imu_timestamps = xrt.get_body_joints_timestamp()
    print(f"IMU timestamps: {imu_timestamps}")
    
    # Get body data timestamp
    body_timestamp = xrt.get_body_timestamp_ns()
    print(f"Body data timestamp: {body_timestamp}")
    
    # Example: Get specific joint data (Head joint is index 15)
    head_pose = body_poses[15]  # Head joint
    x, y, z, qx, qy, qz, qw = head_pose
    print(f"Head pose: Position({x:.3f}, {y:.3f}, {z:.3f}) Rotation({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})")
else:
    print("Body tracking data not available. Make sure:")
    print("1. PICO headset is connected")
    print("2. Body tracking is enabled in the control panel")
    print("3. At least two Pico Swift devices are connected and calibrated")

xrt.close()
```

**Body Joint Indices (similar to SMPL, 24 joints total):**
- 0: Pelvis, 1: Left Hip, 2: Right Hip, 3: Spine1, 4: Left Knee, 5: Right Knee
- 6: Spine2, 7: Left Ankle, 8: Right Ankle, 9: Spine3, 10: Left Foot, 11: Right Foot
- 12: Neck, 13: Left Collar, 14: Right Collar, 15: Head, 16: Left Shoulder, 17: Right Shoulder
- 18: Left Elbow, 19: Right Elbow, 20: Left Wrist, 21: Right Wrist, 22: Left Hand, 23: Right Hand
