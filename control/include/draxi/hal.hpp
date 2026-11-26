#ifndef DRAXI_HAL_HPP
#define DRAXI_HAL_HPP

#include <array>
#include "draxi/joints.hpp"

namespace draxi {

struct JointState {
    float position;
    float velocity;
    float torque;
};

struct JointCommand {
    float position;
    float velocity;
    float kp;
    float kd;
    float torqueFF;
};

struct ImuSample {
    float quaternion[4];
    float gyro[3];
    float accel[3];
};

using JointStates = std::array<JointState, kNumJoints>;
using JointCommands = std::array<JointCommand, kNumJoints>;

class Hal {
public:
    virtual ~Hal() = default;

    virtual void readJoints(JointStates& jointStates) = 0;
    virtual void readImu(ImuSample& imuSample) = 0;
    virtual void sendCommands(const JointCommands& jointCommands) = 0;
};

} // namespace draxi

#endif // DRAXI_HAL_HPP