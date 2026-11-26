#ifndef DRAXI_STAND_CONTROLLER_HPP
#define DRAXI_STAND_CONTROLLER_HPP

#include <array>

#include "draxi/hal.hpp"
#include "draxi/joints.hpp"

namespace draxi {

// Holds every joint at a target angle. It only fills in commands: the PD loop itself runs in the
// motor (RobStride MIT mode), or in MujocoHal standing in for it.
class StandController {
public:
    // Hold the pose the robot is in now
    void reset(const JointStates& states);

    // Move one joint's target, clamped to its limits in kJoints
    void setTarget(JointId joint, float position);
    float target(JointId joint) const { return targets_[index(joint)]; }

    void update(const JointStates& states, const ImuSample& imu, float dt, JointCommands& out) const;

    // Gains scale with each motor's peak torque, so the RobStride 02s and 05s feel alike:
    // kp = kpPerNm * maxTorque [Nm/rad], kd = kdPerNm * maxTorque [Nm·s/rad]
    static constexpr float kpPerNm = 4.0f;
    static constexpr float kdPerNm = 0.1f;

private:
    static constexpr size_t index(JointId joint) { return static_cast<size_t>(joint); }

    std::array<float, kNumJoints> targets_{};
};

} // namespace draxi

#endif // DRAXI_STAND_CONTROLLER_HPP
