#ifndef DRAXI_MUJOCO_HAL_HPP
#define DRAXI_MUJOCO_HAL_HPP

#include <array>

#include <mujoco/mujoco.h>

#include "draxi/hal.hpp"
#include "draxi/joints.hpp"

namespace draxi {

// The Hal on top of MuJoCo. Joints are found by name from kJoints, so the model's order doesn't
// matter; the constructor throws std::runtime_error if the model and kJoints disagree.
//
// It also stands in for the RobStride motors: sendCommands only stores the commands, and
// applyMotors() runs the motor's MIT-mode PD law on the current state. Call it before every
// mj_step, the way the motor runs its loop much faster than the controller.
class MujocoHal : public Hal {
public:
    MujocoHal(const mjModel* m, mjData* d);

    void readJoints(JointStates& jointStates) override;
    void readImu(ImuSample& imuSample) override;
    void sendCommands(const JointCommands& jointCommands) override;

    void applyMotors();

private:
    int sensorAdr(const char* name, int dim) const;

    const mjModel* m_;
    mjData* d_;
    std::array<int, kNumJoints> actuator_{};
    std::array<int, kNumJoints> qposAdr_{};
    std::array<int, kNumJoints> dofAdr_{};
    int quatAdr_, gyroAdr_, accelAdr_;
    JointCommands commands_{};  // all zero gains until the first sendCommands: motors limp
};

} // namespace draxi

#endif // DRAXI_MUJOCO_HAL_HPP
