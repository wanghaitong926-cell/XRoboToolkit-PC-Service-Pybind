// Python.h must come first. Py_LIMITED_API is defined by CMake for abi3 builds.
#include <Python.h>

#include <thread>
#include <iostream>
#include <mutex>
#include <new>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
#include <nlohmann/json.hpp>
#include "PXREARobotSDK.h"
#include "input_state.h"


using json = nlohmann::json;
InputLifecycle input;
// Do not hold input.mutex while joining SDK threads that deliver callbacks.
std::mutex sdkLifecycleMutex;

template <std::size_t N>
std::array<double, N> stringToArray(const std::string& text) {
    std::array<double, N> result{};
    std::stringstream ss(text);
    std::string value;
    std::size_t i = 0;
    while (std::getline(ss, value, ',')) {
        if (i >= N) {
            throw std::invalid_argument("Too many components in XR vector");
        }
        std::size_t used = 0;
        double component = std::stod(value, &used);
        if (used != value.size() || !std::isfinite(component)) {
            throw std::invalid_argument("Invalid component in XR vector");
        }
        result[i++] = component;
    }
    if (i != N) {
        throw std::invalid_argument("Incomplete XR vector");
    }
    return result;
}

std::array<double, 7> stringToPoseArray(const std::string& text) {
    // Tracking quality is modality-specific; inactive hands can contain zero
    // poses. Receipt validity is not a guarantee of usable tracking.
    return stringToArray<7>(text);
}

#include "input_parser.h"

void OnPXREAClientCallback(void*, PXREAClientCallbackType type, int, void* userData) {
    std::lock_guard<std::mutex> lock(input.mutex);
    if (!input.initialized) {
        return;  // Late callbacks during/after close cannot repopulate the cache.
    }
    if (type == PXREAServerConnect) {
        input.serviceConnected = true;
    } else if (type == PXREAServerDisconnect) {
        input.serviceConnected = false;
        input.invalidate("server_disconnected");
    } else if (type == PXREADeviceMissing) {
        if (userData && input.deviceId == static_cast<const char*>(userData)) {
            input.invalidate("device_missing");
        }
    } else if (type == PXREADeviceStateJson && userData) {
        const auto& dsj = *static_cast<PXREADevStateJson*>(userData);
        const std::string deviceId(dsj.devID,
            std::find(std::begin(dsj.devID), std::end(dsj.devID), '\0'));
        if (deviceId.empty() || (!input.deviceId.empty() && input.deviceId != deviceId)) {
            return;
        }
        try {
            const auto outer = json::parse(std::begin(dsj.stateJson),
                std::find(std::begin(dsj.stateJson), std::end(dsj.stateJson), '\0'));
            const auto value = json::parse(outer.at("value").get<std::string>());
            auto next = parseInput(value);
            input.expire();
            if (input.deviceId.empty()) {
                input.deviceId = deviceId;  // Compatibility: pin first parsed frame.
                ++input.generation;
            }
            input.cache = std::move(next);
            input.lastFrame = InputLifecycle::Clock::now();
            input.receivedFrame = true;
            input.valid = true;
            input.status = "active";
        } catch (const std::exception& e) {
            if (input.deviceId == deviceId) {
                input.invalidate("invalid_frame");
            }
            std::cerr << "XR frame rejected: " << e.what() << std::endl;
        }
    }
}

void init() {
    std::lock_guard<std::mutex> sdkLock(sdkLifecycleMutex);
    {
        std::lock_guard<std::mutex> lock(input.mutex);
        if (input.initialized) {
            throw std::runtime_error("PXREARobot SDK is already initialized");
        }
        input.invalidate("waiting");
        input.serviceConnected = false;
        input.initialized = true;
    }
    if (PXREAInit(NULL, OnPXREAClientCallback, PXREAFullMask) != 0) {
        std::lock_guard<std::mutex> lock(input.mutex);
        input.initialized = false;
        input.invalidate("init_failed");
        throw std::runtime_error("PXREAInit failed");
    }
}

void deinit() {
    std::lock_guard<std::mutex> sdkLock(sdkLifecycleMutex);
    bool wasInitialized;
    {
        std::lock_guard<std::mutex> lock(input.mutex);
        wasInitialized = input.initialized;
        input.initialized = false;
        input.serviceConnected = false;
        input.invalidate("closed");
    }
    if (wasInitialized) {
        PXREADeinit();
    }
}

template <typename T>
T readInput(T CachedInput::*member) {
    std::lock_guard<std::mutex> lock(input.mutex);
    input.expire();
    return input.cache.*member;
}

// Preserve legacy getter signatures and return shapes.
std::array<double, 7> getLeftControllerPose() { return readInput(&CachedInput::LeftControllerPose); }
std::array<double, 7> getRightControllerPose() { return readInput(&CachedInput::RightControllerPose); }
std::array<double, 7> getHeadsetPose() { return readInput(&CachedInput::HeadsetPose); }
double getLeftTrigger() { return readInput(&CachedInput::LeftTrigger); }
double getLeftGrip() { return readInput(&CachedInput::LeftGrip); }
double getRightTrigger() { return readInput(&CachedInput::RightTrigger); }
double getRightGrip() { return readInput(&CachedInput::RightGrip); }
bool getLeftMenuButton() { return readInput(&CachedInput::LeftMenuButton); }
bool getRightMenuButton() { return readInput(&CachedInput::RightMenuButton); }
bool getLeftAxisClick() { return readInput(&CachedInput::LeftAxisClick); }
bool getRightAxisClick() { return readInput(&CachedInput::RightAxisClick); }
std::array<double, 2> getLeftAxis() { return readInput(&CachedInput::LeftAxis); }
std::array<double, 2> getRightAxis() { return readInput(&CachedInput::RightAxis); }
bool getLeftPrimaryButton() { return readInput(&CachedInput::LeftPrimaryButton); }
bool getRightPrimaryButton() { return readInput(&CachedInput::RightPrimaryButton); }
bool getLeftSecondaryButton() { return readInput(&CachedInput::LeftSecondaryButton); }
bool getRightSecondaryButton() { return readInput(&CachedInput::RightSecondaryButton); }
int64_t getTimeStampNs() { return readInput(&CachedInput::TimeStampNs); }
std::array<std::array<double, 7>, 26> getLeftHandTrackingState() {
    return readInput(&CachedInput::LeftHandTrackingState);
}
int getLeftHandScale() { return readInput(&CachedInput::LeftHandScale); }
int getLeftHandIsActive() { return readInput(&CachedInput::LeftHandIsActive); }
std::array<std::array<double, 7>, 26> getRightHandTrackingState() {
    return readInput(&CachedInput::RightHandTrackingState);
}
int getRightHandScale() { return readInput(&CachedInput::RightHandScale); }
int getRightHandIsActive() { return readInput(&CachedInput::RightHandIsActive); }
bool isBodyDataAvailable() { return readInput(&CachedInput::BodyDataAvailable); }
std::array<std::array<double, 7>, 24> getBodyJointsPose() {
    return readInput(&CachedInput::BodyJointsPose);
}
std::array<std::array<double, 6>, 24> getBodyJointsVelocity() {
    return readInput(&CachedInput::BodyJointsVelocity);
}
std::array<std::array<double, 6>, 24> getBodyJointsAcceleration() {
    return readInput(&CachedInput::BodyJointsAcceleration);
}
std::array<int64_t, 24> getBodyJointsTimestamp() { return readInput(&CachedInput::BodyJointsTimestamp); }
int64_t getBodyTimeStampNs() { return readInput(&CachedInput::BodyTimeStampNs); }
int numMotionDataAvailable() { return readInput(&CachedInput::NumMotionDataAvailable); }

template <typename T>
std::vector<T> readMotion(std::array<T, 3> CachedInput::*member) {
    std::lock_guard<std::mutex> lock(input.mutex);
    input.expire();
    const auto& values = input.cache.*member;
    return {values.begin(), values.begin() + input.cache.NumMotionDataAvailable};
}

std::vector<std::array<double, 7>> getMotionTrackerPose() {
    return readMotion(&CachedInput::MotionTrackerPose);
}
std::vector<std::array<double, 6>> getMotionTrackerVelocity() {
    return readMotion(&CachedInput::MotionTrackerVelocity);
}
std::vector<std::array<double, 6>> getMotionTrackerAcceleration() {
    return readMotion(&CachedInput::MotionTrackerAcceleration);
}
std::vector<std::string> getMotionTrackerSerialNumbers() {
    return readMotion(&CachedInput::MotionTrackerSerialNumbers);
}
int64_t getMotionTimeStampNs() { return readInput(&CachedInput::MotionTimeStampNs); }

void setDeviceId(const std::string& deviceId) {
    if (deviceId.size() >= 32 || deviceId.find('\0') != std::string::npos) {
        throw std::invalid_argument("device_id must contain at most 31 bytes and no NUL");
    }
    std::lock_guard<std::mutex> lock(input.mutex);
    input.deviceId = deviceId;
    input.invalidate(input.initialized ? "waiting" : "uninitialized");
}

void setInputTimeoutMs(int timeoutMs) {
    if (timeoutMs <= 0) {
        throw std::invalid_argument("timeout_ms must be positive");
    }
    std::lock_guard<std::mutex> lock(input.mutex);
    // Increasing a deadline must not revive data that already expired.
    input.expire();
    input.timeoutMs = timeoutMs;
    input.expire();
}

bool isInputValid() {
    std::lock_guard<std::mutex> lock(input.mutex);
    input.expire();
    return input.valid;
}

json getInputSnapshot() {
    std::lock_guard<std::mutex> lock(input.mutex);
    input.expire();
    json age = nullptr;
    if (input.receivedFrame) {
        age = std::chrono::duration<double, std::milli>(
            InputLifecycle::Clock::now() - input.lastFrame).count();
    }
    return {
        {"device_id", input.deviceId}, {"valid", input.valid},
        {"status", input.status}, {"generation", input.generation},
        {"service_connected", input.serviceConnected}, {"age_ms", age},
        {"timeout_ms", input.timeoutMs}, {"timestamp_ns", input.cache.TimeStampNs},
        {"data", input.cache.data}
    };
}

int DeviceControlJsonWrapper(const std::string& dev_id, const std::string& json_str) {
    const int rc = PXREADeviceControlJson(dev_id.c_str(), json_str.c_str());
    if (rc != 0) {
        throw std::runtime_error("device_control_json failed");
    }
    return rc; // 0
}

int SendBytesToDeviceWrapper(const std::string& dev_id, const std::string& blob) {
    const int rc = PXREASendBytesToDevice(dev_id.c_str(), blob.data(), static_cast<unsigned>(blob.size()));
    if (rc != 0) {
        throw std::runtime_error("send_bytes_to_device failed");
    }
    return rc; // 0
}


// ---------------------------------------------------------------------------
// Python module
//
// Written against the CPython Limited API instead of pybind11, which cannot
// target it: the resulting abi3 wheel installs on every CPython >= 3.10. The
// conversions match what pybind11 produced (std::array/std::vector -> list).
// ---------------------------------------------------------------------------

namespace {

PyObject* toPython(const json& value) {
    // Use public Limited API calls, retaining the cp310-abi3 wheel target.
    const auto serialized = value.dump();
    PyObject* module = PyImport_ImportModule("json");
    if (module == nullptr) {
        return nullptr;
    }
    PyObject* loads = PyObject_GetAttrString(module, "loads");
    Py_DECREF(module);
    if (loads == nullptr) {
        return nullptr;
    }
    PyObject* result = PyObject_CallFunction(loads, "s", serialized.c_str());
    Py_DECREF(loads);
    return result;
}

template <typename T>
PyObject* toPython(const T& value) {
    if constexpr (std::is_same_v<T, bool>) {
        return PyBool_FromLong(value);
    } else if constexpr (std::is_integral_v<T>) {
        return PyLong_FromLongLong(value);
    } else if constexpr (std::is_floating_point_v<T>) {
        return PyFloat_FromDouble(value);
    } else if constexpr (std::is_same_v<T, std::string>) {
        return PyUnicode_FromStringAndSize(value.data(), static_cast<Py_ssize_t>(value.size()));
    } else {
        PyObject* list = PyList_New(static_cast<Py_ssize_t>(value.size()));
        if (list == nullptr) {
            return nullptr;
        }
        Py_ssize_t i = 0;
        for (const auto& item : value) {
            PyObject* element = toPython(item);
            if (element == nullptr) {
                Py_DECREF(list);
                return nullptr;
            }
            PyList_SetItem(list, i++, element);  // steals the reference
        }
        return list;
    }
}

// C++ exceptions must not unwind into the interpreter; raise them as Python
// exceptions the way pybind11 did.
template <typename Body>
PyObject* translateExceptions(Body body) {
    try {
        return body();
    } catch (const std::bad_alloc&) {
        return PyErr_NoMemory();
    } catch (const std::exception& e) {
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return nullptr;
    }
}

template <auto Fn>
PyObject* callNoArgs(PyObject* /*module*/, PyObject* /*unused*/) {
    return translateExceptions([]() -> PyObject* {
        if constexpr (std::is_void_v<decltype(Fn())>) {
            Fn();
            Py_RETURN_NONE;
        } else {
            return toPython(Fn());
        }
    });
}

// str (UTF-8 encoded), bytes or bytearray -> std::string, like pybind11's
// std::string caster. Deliberately not PyArg_ParseTuple's "s#"/"y#": when
// compiled against Python >= 3.13 headers, '#' formats raise SystemError
// (PY_SSIZE_T_CLEAN) once the abi3 module runs on Python 3.10-3.12.
bool toStdString(PyObject* obj, std::string& out) {
    const char* data;
    Py_ssize_t size;
    if (PyUnicode_Check(obj)) {
        data = PyUnicode_AsUTF8AndSize(obj, &size);
        if (data == nullptr) {
            return false;
        }
    } else if (PyBytes_Check(obj)) {
        char* buffer;
        if (PyBytes_AsStringAndSize(obj, &buffer, &size) < 0) {
            return false;
        }
        data = buffer;
    } else if (PyByteArray_Check(obj)) {
        data = PyByteArray_AsString(obj);
        size = PyByteArray_Size(obj);
    } else {
        PyErr_Format(PyExc_TypeError, "expected str, bytes or bytearray, got %R", reinterpret_cast<PyObject*>(Py_TYPE(obj)));
        return false;
    }
    out.assign(data, static_cast<size_t>(size));
    return true;
}

PyObject* pySetDeviceId(PyObject*, PyObject* arg) {
    if (!PyUnicode_Check(arg)) {
        PyErr_SetString(PyExc_TypeError, "device_id must be a string");
        return nullptr;
    }
    return translateExceptions([&]() -> PyObject* {
        std::string deviceId;
        if (!toStdString(arg, deviceId)) {
            return nullptr;
        }
        setDeviceId(deviceId);
        Py_RETURN_NONE;
    });
}

PyObject* pySetInputTimeoutMs(PyObject*, PyObject* arg) {
    long timeout = PyLong_AsLong(arg);
    if (PyErr_Occurred()) {
        return nullptr;
    }
    if (timeout <= 0 || timeout > 2147483647L) {
        PyErr_SetString(PyExc_ValueError, "timeout_ms must be between 1 and 2147483647");
        return nullptr;
    }
    return translateExceptions([&]() -> PyObject* {
        setInputTimeoutMs(static_cast<int>(timeout));
        Py_RETURN_NONE;
    });
}

PyObject* pyDeviceControlJson(PyObject* /*module*/, PyObject* args) {
    PyObject* devId;
    PyObject* jsonStr;
    if (!PyArg_ParseTuple(args, "OO:device_control_json", &devId, &jsonStr)) {
        return nullptr;
    }
    return translateExceptions([&]() -> PyObject* {
        std::string devIdValue;
        std::string jsonValue;
        if (!toStdString(devId, devIdValue) || !toStdString(jsonStr, jsonValue)) {
            return nullptr;
        }
        return toPython(DeviceControlJsonWrapper(devIdValue, jsonValue));
    });
}

PyObject* pySendBytesToDevice(PyObject* /*module*/, PyObject* args) {
    PyObject* devId;
    PyObject* data;  // "S": bytes only, like pybind11's py::bytes
    if (!PyArg_ParseTuple(args, "OS:send_bytes_to_device", &devId, &data)) {
        return nullptr;
    }
    return translateExceptions([&]() -> PyObject* {
        std::string devIdValue;
        std::string blob;
        if (!toStdString(devId, devIdValue) || !toStdString(data, blob)) {
            return nullptr;
        }
        return toPython(SendBytesToDeviceWrapper(devIdValue, blob));
    });
}

// Docstrings keep pybind11's layout: "name() -> type", blank line, description.
#define XRT_FUNCTION(name, fn, returns, doc) \
    { name, callNoArgs<fn>, METH_NOARGS, name "() -> " returns "\n\n" doc }

PyMethodDef kMethods[] = {
    XRT_FUNCTION("init", init, "None", "Initialize the PXREARobot SDK."),
    XRT_FUNCTION("close", deinit, "None", "Deinitialize the PXREARobot SDK."),
    {"set_device_id", pySetDeviceId, METH_O,
     "set_device_id(device_id: str, /) -> None\n\nSelect a device and invalidate cached input; empty string enables first-frame binding."},
    {"set_input_timeout_ms", pySetInputTimeoutMs, METH_O,
     "set_input_timeout_ms(timeout_ms: int, /) -> None\n\nSet the positive monotonic frame receipt deadline (default 500 ms)."},
    XRT_FUNCTION("is_input_valid", isInputValid, "bool", "Whether selected input has a parsed, unexpired frame."),
    XRT_FUNCTION("get_input_snapshot", getInputSnapshot, "dict", "Get one coherent frame with device, validity and lifecycle metadata."),
    XRT_FUNCTION("get_left_controller_pose", getLeftControllerPose, "list[float]", "Get the left controller pose."),
    XRT_FUNCTION("get_right_controller_pose", getRightControllerPose, "list[float]", "Get the right controller pose."),
    XRT_FUNCTION("get_headset_pose", getHeadsetPose, "list[float]", "Get the headset pose."),
    XRT_FUNCTION("get_left_trigger", getLeftTrigger, "float", "Get the left trigger value."),
    XRT_FUNCTION("get_left_grip", getLeftGrip, "float", "Get the left grip value."),
    XRT_FUNCTION("get_right_trigger", getRightTrigger, "float", "Get the right trigger value."),
    XRT_FUNCTION("get_right_grip", getRightGrip, "float", "Get the right grip value."),
    XRT_FUNCTION("get_left_menu_button", getLeftMenuButton, "bool", "Get the left menu button state."),
    XRT_FUNCTION("get_right_menu_button", getRightMenuButton, "bool", "Get the right menu button state."),
    XRT_FUNCTION("get_left_axis_click", getLeftAxisClick, "bool", "Get the left axis click state."),
    XRT_FUNCTION("get_right_axis_click", getRightAxisClick, "bool", "Get the right axis click state."),
    XRT_FUNCTION("get_left_axis", getLeftAxis, "list[float]", "Get the left axis values (x, y)."),
    XRT_FUNCTION("get_right_axis", getRightAxis, "list[float]", "Get the right axis values (x, y)."),
    XRT_FUNCTION("get_X_button", getLeftPrimaryButton, "bool", "Get the left primary button state."),
    XRT_FUNCTION("get_A_button", getRightPrimaryButton, "bool", "Get the right primary button state."),
    XRT_FUNCTION("get_Y_button", getLeftSecondaryButton, "bool", "Get the left secondary button state."),
    XRT_FUNCTION("get_B_button", getRightSecondaryButton, "bool", "Get the right secondary button state."),
    XRT_FUNCTION("get_time_stamp_ns", getTimeStampNs, "int", "Get the timestamp in nanoseconds."),
    XRT_FUNCTION("get_left_hand_tracking_state", getLeftHandTrackingState, "list[list[float]]", "Get the left hand state."),
    XRT_FUNCTION("get_right_hand_tracking_state", getRightHandTrackingState, "list[list[float]]", "Get the right hand state."),
    XRT_FUNCTION("get_left_hand_is_active", getLeftHandIsActive, "int", "Get the left hand tracking quality (0 = low, 1 = high)."),
    XRT_FUNCTION("get_right_hand_is_active", getRightHandIsActive, "int", "Get the right hand tracking quality (0 = low, 1 = high)."),

    // Body tracking functions
    XRT_FUNCTION("is_body_data_available", isBodyDataAvailable, "bool", "Check if body tracking data is available."),
    XRT_FUNCTION("get_body_joints_pose", getBodyJointsPose, "list[list[float]]", "Get the body joints pose data (24 joints, 7 values each: x,y,z,qx,qy,qz,qw)."),
    XRT_FUNCTION("get_body_joints_velocity", getBodyJointsVelocity, "list[list[float]]", "Get the body joints velocity data (24 joints, 6 values each: vx,vy,vz,wx,wy,wz)."),
    XRT_FUNCTION("get_body_joints_acceleration", getBodyJointsAcceleration, "list[list[float]]", "Get the body joints acceleration data (24 joints, 6 values each: ax,ay,az,wax,way,waz)."),
    XRT_FUNCTION("get_body_joints_timestamp", getBodyJointsTimestamp, "list[int]", "Get the body joints IMU timestamp data (24 joints)."),
    XRT_FUNCTION("get_body_timestamp_ns", getBodyTimeStampNs, "int", "Get the body data timestamp in nanoseconds."),

    // Motion tracker functions
    XRT_FUNCTION("num_motion_data_available", numMotionDataAvailable, "int", "Check if motion tracker data is available."),
    XRT_FUNCTION("get_motion_tracker_pose", getMotionTrackerPose, "list[list[float]]", "Get the motion tracker pose data (3 trackers, 7 values each: x,y,z,qx,qy,qz,qw)."),
    XRT_FUNCTION("get_motion_tracker_velocity", getMotionTrackerVelocity, "list[list[float]]", "Get the motion tracker velocity data (3 trackers, 6 values each: vx,vy,vz,wx,wy,wz)."),
    XRT_FUNCTION("get_motion_tracker_acceleration", getMotionTrackerAcceleration, "list[list[float]]", "Get the motion tracker acceleration data (3 trackers, 6 values each: ax,ay,az,wax,way,waz)."),
    XRT_FUNCTION("get_motion_tracker_serial_numbers", getMotionTrackerSerialNumbers, "list[str]", "Get the serial numbers of the motion trackers."),
    XRT_FUNCTION("get_motion_timestamp_ns", getMotionTimeStampNs, "int", "Get the motion data timestamp in nanoseconds."),

    // send json bytes functions
    {"device_control_json", pyDeviceControlJson, METH_VARARGS,
     "device_control_json(dev_id: str, json_str: str, /) -> int\n\nSend a JSON control command to a device"},
    {"send_bytes_to_device", pySendBytesToDevice, METH_VARARGS,
     "send_bytes_to_device(dev_id: str, data: bytes, /) -> int\n\nSend raw bytes to a device"},
    {nullptr, nullptr, 0, nullptr},
};

#undef XRT_FUNCTION

PyModuleDef kModule = {
    PyModuleDef_HEAD_INIT,
    "_core",
    "Python bindings for PXREARobot SDK.",
    -1,  // the SDK state above is process-global
    kMethods,
};

}  // namespace

PyMODINIT_FUNC PyInit__core() {
    return PyModule_Create(&kModule);
}
