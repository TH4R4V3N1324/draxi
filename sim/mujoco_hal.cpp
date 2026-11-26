#include "mujoco_hal.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace draxi {

namespace {

constexpr double kRangeTolerance = 1e-3;  // rad, or Nm for torque limits

bool near(double a, double b) { return std::abs(a - b) < kRangeTolerance; }

}  // namespace

MujocoHal::MujocoHal(const mjModel* m, mjData* d) : m_(m), d_(d) {
    for (size_t i = 0; i < kNumJoints; ++i) {
        const JointInfo& info = kJoints[i];
        const std::string name = info.name;

        int act = mj_name2id(m, mjOBJ_ACTUATOR, info.name);
        if (act < 0) throw std::runtime_error("model has no actuator '" + name + "'");
        if (m->actuator_trntype[act] != mjTRN_JOINT) {
            throw std::runtime_error("actuator '" + name + "' does not drive a joint");
        }
        int joint = m->actuator_trnid[2 * act];

        // kJoints and the model must agree, or the firmware and the sim are different robots
        const mjtNum* range = m->jnt_range + 2 * joint;
        if (!near(range[0], info.minRad) || !near(range[1], info.maxRad)) {
            throw std::runtime_error("joint '" + name + "' range [" + std::to_string(range[0]) + ", " +
                                     std::to_string(range[1]) + "] rad does not match kJoints [" +
                                     std::to_string(info.minRad) + ", " + std::to_string(info.maxRad) + "]");
        }
        const mjtNum* ctrl = m->actuator_ctrlrange + 2 * act;
        if (!near(ctrl[0], -info.maxTorque) || !near(ctrl[1], info.maxTorque)) {
            throw std::runtime_error("actuator '" + name + "' ctrlrange does not match kJoints maxTorque " +
                                     std::to_string(info.maxTorque));
        }

        actuator_[i] = act;
        qposAdr_[i] = m->jnt_qposadr[joint];
        dofAdr_[i] = m->jnt_dofadr[joint];
    }

    quatAdr_ = sensorAdr("imu_quat", 4);
    gyroAdr_ = sensorAdr("imu_gyro", 3);
    accelAdr_ = sensorAdr("imu_acc", 3);
}

int MujocoHal::sensorAdr(const char* name, int dim) const {
    int id = mj_name2id(m_, mjOBJ_SENSOR, name);
    if (id < 0) throw std::runtime_error(std::string("model has no sensor '") + name + "'");
    if (m_->sensor_dim[id] != dim) throw std::runtime_error(std::string("sensor '") + name + "' has the wrong size");
    return m_->sensor_adr[id];
}

void MujocoHal::readJoints(JointStates& jointStates) {
    for (size_t i = 0; i < kNumJoints; ++i) {
        jointStates[i].position = static_cast<float>(d_->qpos[qposAdr_[i]]);
        jointStates[i].velocity = static_cast<float>(d_->qvel[dofAdr_[i]]);
        jointStates[i].torque = static_cast<float>(d_->actuator_force[actuator_[i]]);
    }
}

void MujocoHal::readImu(ImuSample& imuSample) {
    const mjtNum* s = d_->sensordata;
    for (int k = 0; k < 4; ++k) imuSample.quaternion[k] = static_cast<float>(s[quatAdr_ + k]);  // w, x, y, z
    for (int k = 0; k < 3; ++k) imuSample.gyro[k] = static_cast<float>(s[gyroAdr_ + k]);
    for (int k = 0; k < 3; ++k) imuSample.accel[k] = static_cast<float>(s[accelAdr_ + k]);
}

void MujocoHal::sendCommands(const JointCommands& jointCommands) { commands_ = jointCommands; }

void MujocoHal::applyMotors() {
    for (size_t i = 0; i < kNumJoints; ++i) {
        const JointCommand& c = commands_[i];
        const mjtNum q = d_->qpos[qposAdr_[i]];
        const mjtNum qd = d_->qvel[dofAdr_[i]];
        const mjtNum torque = c.kp * (c.position - q) + c.kd * (c.velocity - qd) + c.torqueFF;
        const mjtNum limit = kJoints[i].maxTorque;
        d_->ctrl[actuator_[i]] = std::clamp(torque, -limit, limit);
    }
}

} // namespace draxi
