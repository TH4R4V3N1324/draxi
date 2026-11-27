#ifndef DRAXI_JOINTS_HPP
#define DRAXI_JOINTS_HPP

#include <cstdint>
#include <cstddef>
#include <array>

namespace draxi {

enum class JointId : uint8_t {
  HipYawL, HipRollL, HipPitchL, KneeL, AnklePitchL, AnkleRollL,
  HipYawR, HipRollR, HipPitchR, KneeR, AnklePitchR, AnkleRollR,
  NeckYaw, NeckPitch, NeckNod, HeadRoll,
  Count
};

constexpr size_t kNumJoints = static_cast<size_t>(JointId::Count);

constexpr float kPi = 3.14159265358979f;

constexpr float deg(float degrees) {
    return degrees * kPi / 180.0f;
}

struct JointInfo {
    const char* name;
    float minRad, maxRad;
    float maxTorque;
};

// Same order as JointId. Names, ranges and torques match cad/draxi.xml
// (17 Nm = RobStride 02 peak, 5.5 Nm = RobStride 05 peak).
constexpr std::array<JointInfo, kNumJoints> kJoints = {{
    {"hip_yaw_L",   deg(-30), deg(30),  5.5f},
    {"hip_roll_L",  deg(-20), deg(20),  17.0f},
    {"hip_pitch_L", deg(-62), deg(90),  17.0f},
    {"knee_L",      deg(-80), deg(15),  17.0f},
    {"ankle_pitch_L", deg(-60), deg(30), 5.5f},
    {"ankle_roll_L", deg(-30), deg(30), 5.5f},
    {"hip_yaw_R",   deg(-30), deg(30),  5.5f},
    {"hip_roll_R",  deg(-20), deg(20),  17.0f},
    {"hip_pitch_R", deg(-62), deg(90),  17.0f},
    {"knee_R",      deg(-80), deg(15),  17.0f},
    {"ankle_pitch_R", deg(-60), deg(30), 5.5f},
    {"ankle_roll_R", deg(-30), deg(30), 5.5f},
    {"neck_yaw",    deg(-60), deg(60),  5.5f},
    {"neck_pitch",  deg(-90), deg(30),  5.5f},
    {"neck_nod",    deg(-60), deg(90),  5.5f},
    {"head_roll",   deg(-35), deg(35),  5.5f},
}};

} // namespace draxi
#endif // DRAXI_JOINTS_HPP