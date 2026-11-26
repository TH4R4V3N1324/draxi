#include "draxi/stand_controller.hpp"

#include <algorithm>

namespace draxi {

void StandController::reset(const JointStates& states) {
    for (size_t i = 0; i < kNumJoints; ++i) {
        setTarget(static_cast<JointId>(i), states[i].position);
    }
}

void StandController::setTarget(JointId joint, float position) {
    const JointInfo& info = kJoints[index(joint)];
    targets_[index(joint)] = std::clamp(position, info.minRad, info.maxRad);
}

void StandController::update(const JointStates& /*states*/, const ImuSample& /*imu*/, float /*dt*/,
                             JointCommands& out) const {
    for (size_t i = 0; i < kNumJoints; ++i) {
        const float maxTorque = kJoints[i].maxTorque;
        out[i].position = targets_[i];
        out[i].velocity = 0.0f;
        out[i].kp = kpPerNm * maxTorque;
        out[i].kd = kdPerNm * maxTorque;
        out[i].torqueFF = 0.0f;
    }
}

} // namespace draxi
