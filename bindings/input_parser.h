#pragma once

// Included after stringToArray / stringToPoseArray in py_bindings.cpp.
// Build a new frame before publication so parse failures cannot partially
// overwrite the currently visible controller/hand/body state.
CachedInput parseInput(const json& value) {
    if (!value.is_object() ||
        !(value.contains("Controller") || value.contains("Head") ||
          value.contains("Hand") || value.contains("Body") || value.contains("Motion"))) {
        throw std::invalid_argument("XR frame contains no tracking data");
    }
    CachedInput next;
    next.data = value;
    if (value.contains("Controller")) {
        const auto& controllers = value.at("Controller");
        if (controllers.contains("left")) {
            const auto& left = controllers.at("left");
            next.LeftControllerPose = stringToPoseArray(left.at("pose").get<std::string>());
            next.LeftTrigger = left.at("trigger").get<double>();
            next.LeftGrip = left.at("grip").get<double>();
            next.LeftMenuButton = left.at("menuButton").get<bool>();
            next.LeftAxis = {left.at("axisX").get<double>(), left.at("axisY").get<double>()};
            next.LeftAxisClick = left.at("axisClick").get<bool>();
            next.LeftPrimaryButton = left.at("primaryButton").get<bool>();
            next.LeftSecondaryButton = left.at("secondaryButton").get<bool>();
        }
        if (controllers.contains("right")) {
            const auto& right = controllers.at("right");
            next.RightControllerPose = stringToPoseArray(right.at("pose").get<std::string>());
            next.RightTrigger = right.at("trigger").get<double>();
            next.RightGrip = right.at("grip").get<double>();
            next.RightMenuButton = right.at("menuButton").get<bool>();
            next.RightAxis = {right.at("axisX").get<double>(), right.at("axisY").get<double>()};
            next.RightAxisClick = right.at("axisClick").get<bool>();
            next.RightPrimaryButton = right.at("primaryButton").get<bool>();
            next.RightSecondaryButton = right.at("secondaryButton").get<bool>();
        }
    }
    if (value.contains("Head")) {
        next.HeadsetPose = stringToPoseArray(value.at("Head").at("pose").get<std::string>());
    }
    next.TimeStampNs = value.value("timeStampNs", int64_t{0});
    if (value.contains("Hand")) {
        const auto& hands = value.at("Hand");
        if (hands.contains("leftHand")) {
            const auto& hand = hands.at("leftHand");
            next.LeftHandScale = hand.at("scale").get<double>();
            next.LeftHandIsActive = hand.at("isActive").get<int>();
            for (int i = 0; i < 26; ++i) {
                next.LeftHandTrackingState[i] = stringToPoseArray(
                    hand.at("HandJointLocations").at(i).at("p").get<std::string>());
            }
        }
        if (hands.contains("rightHand")) {
            const auto& hand = hands.at("rightHand");
            next.RightHandScale = hand.at("scale").get<double>();
            next.RightHandIsActive = hand.at("isActive").get<int>();
            for (int i = 0; i < 26; ++i) {
                next.RightHandTrackingState[i] = stringToPoseArray(
                    hand.at("HandJointLocations").at(i).at("p").get<std::string>());
            }
        }
    }
    if (value.contains("Body")) {
        const auto& body = value.at("Body");
        next.BodyTimeStampNs = body.value("timeStampNs", int64_t{0});
        if (body.contains("joints") && body.at("joints").is_array()) {
            const auto& joints = body.at("joints");
            int count = std::min(static_cast<int>(joints.size()), 24);
            for (int i = 0; i < count; ++i) {
                const auto& joint = joints.at(i);
                if (joint.contains("p"))
                    next.BodyJointsPose[i] = stringToPoseArray(joint.at("p").get<std::string>());
                if (joint.contains("va"))
                    next.BodyJointsVelocity[i] = stringToArray<6>(joint.at("va").get<std::string>());
                if (joint.contains("wva"))
                    next.BodyJointsAcceleration[i] = stringToArray<6>(joint.at("wva").get<std::string>());
                next.BodyJointsTimestamp[i] = joint.value("t", int64_t{0});
            }
            next.BodyDataAvailable = count > 0;
        }
    }
    if (value.contains("Motion")) {
        const auto& motion = value.at("Motion");
        next.MotionTimeStampNs = motion.value("timeStampNs", int64_t{0});
        if (motion.contains("joints") && motion.at("joints").is_array()) {
            const auto& joints = motion.at("joints");
            next.NumMotionDataAvailable = std::min(static_cast<int>(joints.size()), 3);
            for (int i = 0; i < next.NumMotionDataAvailable; ++i) {
                const auto& joint = joints.at(i);
                if (joint.contains("p"))
                    next.MotionTrackerPose[i] = stringToPoseArray(joint.at("p").get<std::string>());
                if (joint.contains("va"))
                    next.MotionTrackerVelocity[i] = stringToArray<6>(joint.at("va").get<std::string>());
                if (joint.contains("wva"))
                    next.MotionTrackerAcceleration[i] = stringToArray<6>(joint.at("wva").get<std::string>());
                next.MotionTrackerSerialNumbers[i] = joint.value("sn", std::string{});
            }
        }
    }
    return next;
}
