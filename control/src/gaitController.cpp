#include "draxi/gaitController.hpp"

#include <algorithm>
#include <cmath>

namespace draxi {

namespace {

// The neck's joints, body to head, as the robot's appendage NECK
constexpr JointId kNeck[4] = {JointId::NeckYaw, JointId::NeckPitch, JointId::NeckNod, JointId::HeadRoll};

// Leg joint j of a leg in kJoints / JointStates / JointCommands: left leg 0-5, right leg 6-11
constexpr size_t jointIndex(int leg, int j) { return static_cast<size_t>(leg * DraxiKinematics::kLegJoints + j); }

}  // namespace

gait_core::GaitParams GaitController::defaultParams() {
    const RobotDefinition r = robot();
    gait_core::GaitParams p;
    // A biped stands on one foot while the other swings, which gait_core's static check calls
    // unstable; it sways the body over the planted foot instead
    p.allow_unstable_gaits = true;
    // The "seated" pose is a crouch with the feet where they stand: startup moves the legs there,
    // then stands up, without stepping the feet anywhere
    p.sit_reach = r.standReach;
    p.sit_height = Scalar(0.13);
    p.stand_on_startup = true;
    p.max_height = -r.standHeight + Scalar(0.01);  // the default 0.16 m is just below Draxi's stance

    // Walking: one foot can't hold the centre of mass over it (the hip roll stops at 20 degrees),
    // but it can hold the ZMP, so preview control plans the weight from foot to foot through the
    // gait's both-feet-down phases. Torque feedforward carries the weight (without it the sim falls
    // in under a second of walking).
    p.preview_control = true;
    p.preview_dynamic_gaits = true;
    p.torque_output = true;
    // Quick steps: the quicker the feet swap, the less the body has to sway to stay over them (+-30 mm
    // at 0.16 s phases against +-55 mm at 0.3 s, about 3 steps a second). Tuned in the sim at 0.01-0.03
    // m/s; 0.14 s phases or a 30 mm step fall. Needs the velocity targets in update() to keep up.
    p.phase_duration = Scalar(0.16);
    p.min_phase_duration = Scalar(0.1);
    p.step_height = Scalar(0.025);
    // Flat floors only for now: on contact at these speeds, terrain adaptation cuts swings short and
    // lifts the "stumbling" foot, which breaks up the stepping rhythm
    p.terrain_adaptation = false;
    // Strafe mode, so setVelocity's vy steps sideways (in normal mode gait_core turns with it instead)
    p.initial_mode = gait_core::MODE_STRAFE;
    // The soles slip round a little as the weight passes from foot to foot, turning Draxi ~23% further
    // than asked in the sim: turn feedback learns that from the IMU and takes it off (101-103% after)
    p.turn_feedback = true;
    return p;
}

GaitController::GaitController(const gait_core::GaitParams& params)
    : params_(params),
      robot_(robot()),
      left_(robot_.legs[LEFT]),
      right_(robot_.legs[RIGHT]),
      engine_(params_, robot_) {
    useOurKinematics();
}

void GaitController::useOurKinematics() {
    // Before the first update: it resets that leg's joint command
    engine_.setLegKinematics(LEFT, &left_);
    engine_.setLegKinematics(RIGHT, &right_);
}

void GaitController::reset(const JointStates& states) {
    auto log = std::move(engine_.log);
    engine_ = gait_core::GaitEngine(params_, robot_);
    engine_.log = std::move(log);
    useOurKinematics();
    for (size_t i = 0; i < kNumJoints; ++i) {
        holdTargets_[i] = states[i].position;
    }
    velocityTargets_.fill(0.0f);
    headStarted_ = false;
}

void GaitController::setVelocity(float vx, float vy, float wz) {
    gait_core::VelocityCommand cmd;
    cmd.vx = vx;
    cmd.vy = vy;
    cmd.wz = wz;
    engine_.setVelocity(cmd);
}

void GaitController::update(const JointStates& states, const ImuSample& imu, float dt, JointCommands& out) {
    now_ += static_cast<gait_core::Timestamp>(std::llround(static_cast<double>(dt) * 1e9));

    // Joints in. JointFeedback's legs default to a hexapod's 3 joints, so size them to ours
    gait_core::JointFeedback feedback;
    feedback.stamp = now_;
    feedback.hasEffort = true;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        feedback.angles[leg].resize(kLegJoints);
        feedback.effort[leg].resize(kLegJoints);
        for (int j = 0; j < kLegJoints; ++j) {
            feedback.angles[leg](j) = states[jointIndex(leg, j)].position;
            feedback.effort[leg](j) = states[jointIndex(leg, j)].torque;
        }
        feedback.present[leg] = true;
    }
    // The neck, for the engine's centre of mass (a head carries nothing, so no payload from its efforts)
    feedback.appendagePresent[NECK] = true;
    feedback.appendageAngles[NECK].resize(4);
    feedback.appendageEffort[NECK].resize(4);
    for (int j = 0; j < 4; ++j) {
        feedback.appendageAngles[NECK](j) = states[static_cast<size_t>(kNeck[j])].position;
        feedback.appendageEffort[NECK](j) = states[static_cast<size_t>(kNeck[j])].torque;
    }
    engine_.onJointStates(feedback);

    // IMU in: the quaternion (w, x, y, z) as roll, pitch and yaw (ZYX)
    const float w = imu.quaternion[0], x = imu.quaternion[1], y = imu.quaternion[2], z = imu.quaternion[3];
    gait_core::ImuSample sample;
    sample.stamp = now_;
    sample.roll = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    sample.pitch = std::asin(std::fmax(-1.0f, std::fmin(1.0f, 2 * (w * y - z * x))));
    sample.yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    sample.angularVelocity = Vec3(imu.gyro[0], imu.gyro[1], imu.gyro[2]);
    sample.linearAcceleration = Vec3(imu.accel[0], imu.accel[1], imu.accel[2]);
    engine_.setImu(sample);

    const gait_core::GaitOutput output = engine_.update(dt, now_);

    // Commands out: every joint held where reset() found it, then the legs from the engine once it
    // commands them (it doesn't while it waits for its first joint states)
    for (size_t i = 0; i < kNumJoints; ++i) {
        const float maxTorque = kJoints[i].maxTorque;
        out[i] = {holdTargets_[i], 0.0f, kpPerNm * maxTorque, kdPerNm * maxTorque, 0.0f};
    }
    if (!output.commanded) {
        velocityTargets_.fill(0.0f);
        return;
    }
    const float smoothing = dt > 0 ? dt / (kVelocityFilter + dt) : 0.0f;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        const LegDefinition& def = robot_.legs[leg];
        for (int j = 0; j < kLegJoints; ++j) {
            const size_t i = jointIndex(leg, j);
            JointCommand& c = out[i];
            // The engine can ask for angles past a joint's limits (its IK solves past them, and its
            // swing and sway don't always check): never send the motor into its hard stop
            const JointInfo& info = kJoints[i];
            c.position = std::clamp(static_cast<float>(output.joints[leg](j)), info.minRad, info.maxRad);
            c.torqueFF = output.hasTorques ? output.torques[leg](j) : 0.0f;

            // Velocity target: how fast the command moves, smoothed and within the joint's speed limit
            const float limit = def.joints[j].velocityLimit(params_.max_joint_speed);
            const float rate = dt > 0 ? std::clamp((c.position - holdTargets_[i]) / dt, -limit, limit) : 0.0f;
            velocityTargets_[i] += smoothing * (rate - velocityTargets_[i]);
            c.velocity = velocityTargets_[i];

            holdTargets_[i] = c.position;  // held there if the engine stops commanding
        }
    }
    headCommands(states, output, dt, out);
}

// The head: towards a target or straight ahead, with the bob on top, within the joints' limits and
// at up to kHeadSpeed; held up against gravity by the engine's appendage torques
void GaitController::headCommands(const JointStates& states, const gait_core::GaitOutput& output, float dt, JointCommands& out) {
    if (!headStarted_) {
        for (int j = 0; j < 4; ++j) headAngles_[j] = states[static_cast<size_t>(kNeck[j])].position;
        headStarted_ = true;
    }
    float want[4] = {0, 0, 0, 0};   // the neck straight out, as in CAD, facing ahead
    if (looking_) {
        // Yaw from the neck's yaw joint; nod from the nod joint, out along the yawed neck
        const gait_core::AppendageDefinition& neck = robot_.appendages[NECK];
        const gait_core::Vec3 base = neck.mount.translation_offset;
        const gait_core::Vec3 d = lookTarget_ - base;
        want[0] = std::atan2(d.y(), d.x());
        const JointInfo& yawInfo = kJoints[static_cast<size_t>(JointId::NeckYaw)];
        const float yaw = std::clamp(want[0], yawInfo.minRad, yawInfo.maxRad);
        const float reach = static_cast<float>(neck.joints[2].origin.x());
        const gait_core::Vec3 nod = base + neck.joints[1].origin + gait_core::Vec3(reach * std::cos(yaw), reach * std::sin(yaw), 0);
        const gait_core::Vec3 e = lookTarget_ - nod;
        want[2] = -std::atan2(e.z(), std::hypot(e.x(), e.y()));   // + nods down
    }
    if (bob_ && output.commanded) {
        // How far the swinging foot is lifted: the two feet's commanded heights apart (leg frames)
        const float zL = static_cast<float>(left_.fk(output.joints[LEFT]).z());
        const float zR = static_cast<float>(right_.fk(output.joints[RIGHT]).z());
        const float stepHeight = std::max(0.01f, static_cast<float>(params_.step_height));
        const float lift = std::clamp(std::abs(zL - zR) / stepHeight, 0.0f, 1.0f);
        want[2] -= kBobNod * lift;                                                 // up as a foot lifts
        want[3] += kBobRoll * std::clamp((zL - zR) / stepHeight, -1.0f, 1.0f);    // towards the foot down
    }
    const float maxStep = kHeadSpeed * dt;
    const bool holding = output.commanded && output.appendageTorques[NECK].size() == 4;
    for (int j = 0; j < 4; ++j) {
        const size_t i = static_cast<size_t>(kNeck[j]);
        const float target = std::clamp(want[j], kJoints[i].minRad, kJoints[i].maxRad);
        const float before = headAngles_[j];
        headAngles_[j] += std::clamp(target - headAngles_[j], -maxStep, maxStep);
        JointCommand& c = out[i];
        c.position = headAngles_[j];
        c.velocity = dt > 0 ? (headAngles_[j] - before) / dt : 0.0f;
        c.torqueFF = holding ? static_cast<float>(output.appendageTorques[NECK](j)) : 0.0f;
        holdTargets_[i] = c.position;
    }
}

} // namespace draxi
