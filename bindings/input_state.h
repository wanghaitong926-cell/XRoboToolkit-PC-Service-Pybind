#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <nlohmann/json.hpp>

// A single cache represents one complete message from one selected device.
// Missing fields start at neutral values instead of surviving from an old frame.
struct CachedInput {
    std::array<double, 7> LeftControllerPose{};
    std::array<double, 7> RightControllerPose{};
    std::array<double, 7> HeadsetPose{};
    std::array<std::array<double, 7>, 26> LeftHandTrackingState{};
    double LeftHandScale = 1.0;
    int LeftHandIsActive = 0;
    std::array<std::array<double, 7>, 26> RightHandTrackingState{};
    double RightHandScale = 1.0;
    int RightHandIsActive = 0;
    std::array<std::array<double, 7>, 24> BodyJointsPose{};
    std::array<std::array<double, 6>, 24> BodyJointsVelocity{};
    std::array<std::array<double, 6>, 24> BodyJointsAcceleration{};
    std::array<int64_t, 24> BodyJointsTimestamp{};
    int64_t BodyTimeStampNs = 0;
    bool BodyDataAvailable = false;
    std::array<std::array<double, 7>, 3> MotionTrackerPose{};
    std::array<std::array<double, 6>, 3> MotionTrackerVelocity{};
    std::array<std::array<double, 6>, 3> MotionTrackerAcceleration{};
    std::array<std::string, 3> MotionTrackerSerialNumbers{};
    int64_t MotionTimeStampNs = 0;
    int NumMotionDataAvailable = 0;
    bool LeftMenuButton = false;
    double LeftTrigger = 0.0;
    double LeftGrip = 0.0;
    std::array<double, 2> LeftAxis{};
    bool LeftAxisClick = false;
    bool LeftPrimaryButton = false;
    bool LeftSecondaryButton = false;
    bool RightMenuButton = false;
    double RightTrigger = 0.0;
    double RightGrip = 0.0;
    std::array<double, 2> RightAxis{};
    bool RightAxisClick = false;
    bool RightPrimaryButton = false;
    bool RightSecondaryButton = false;
    int64_t TimeStampNs = 0;
    nlohmann::json data = nlohmann::json::object();
};

struct InputLifecycle {
    using Clock = std::chrono::steady_clock;
    std::mutex mutex;
    CachedInput cache;
    std::string deviceId;
    std::string status = "uninitialized";
    bool initialized = false;
    bool serviceConnected = false;
    bool valid = false;
    uint64_t generation = 0;
    int timeoutMs = 500;
    Clock::time_point lastFrame{};
    bool receivedFrame = false;

    // Callers hold mutex. Binding survives invalidation: never fail over to
    // another headset because a heartbeat or a frame has timed out.
    void invalidate(const std::string& reason) {
        cache = CachedInput{};
        valid = false;
        receivedFrame = false;
        status = reason;
        ++generation;
    }

    void expire() {
        if (valid && Clock::now() - lastFrame >= std::chrono::milliseconds(timeoutMs)) {
            // Age is about data receipt, not proof of a physical disconnect.
            cache = CachedInput{};
            valid = false;
            status = "stale";
            ++generation;
        }
    }
};
