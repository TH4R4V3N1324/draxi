#ifndef DRAXI_GAIT_CONTROLLER_HPP
#define DRAXI_GAIT_CONTROLLER_HPP

#include <array>

#include <gait_core/locomotion/gait_engine.hpp>

#include "draxi/draxi.hpp"
#include "draxi/draxiKinematics.hpp"
#include "draxi/hal.hpp"
#include "draxi/joints.hpp"

namespace draxi {

// Walks Draxi with gait_core's GaitEngine, using DraxiKinematics for both legs. Like
// StandController it only fills in commands: the leg joints' targets (and feedforward torques,
// with GaitParams::torque_output) come from the engine. The neck is the engine's appendage: its
// joint states go to the engine, which counts the head's weight wherever it is and gives the torques
// that hold it up; the head faces ahead, or looks at a target (lookAt), and can bob with the steps.
// The motors (RobStride MIT mode, or MujocoHal standing in for them) run the PD loop.
class GaitController {
public:
    // Draxi's defaults: a crouch for the seated pose, standing straight after startup
    static gait_core::GaitParams defaultParams();

    explicit GaitController(const gait_core::GaitParams& params = defaultParams());

    // The engine holds pointers to this object's kinematics, so it can't be copied or moved
    GaitController(const GaitController&) = delete;
    GaitController& operator=(const GaitController&) = delete;

    // Start again from the pose the robot is in now: a fresh engine (which runs its startup into
    // the crouch and stands up) and the neck held where it is
    void reset(const JointStates& states);

    // m/s forward, m/s left (strafing), rad/s anticlockwise
    void setVelocity(float vx, float vy, float wz);

    // Look at a point (body frame, m): the neck turns towards it and the head nods up or down to it,
    // within the joints' limits, at up to kHeadSpeed. lookAhead() faces forward again.
    void lookAt(const gait_core::Vec3& target) { lookTarget_ = target; looking_ = true; }
    void lookAhead() { looking_ = false; }
    // Nod and roll the head in time with the steps, for character: up as a foot lifts, back down as
    // it lands, and a little roll towards the foot that's down. Off by default.
    void setBob(bool on) { bob_ = on; }

    void update(const JointStates& states, const ImuSample& imu, float dt, JointCommands& out);

    // For everything else: setGait, sit, setHeight, log, balanceState()...
    gait_core::GaitEngine& engine() { return engine_; }
    const gait_core::GaitEngine& engine() const { return engine_; }

    // kp = kpPerNm * maxTorque [Nm/rad], kd = kdPerNm * maxTorque [Nm·s/rad]. Twice StandController's:
    // in the sim, kp 4 lets the legs sag behind the plan and the walk stalls or falls; 8-10 with kd
    // 0.2-0.3 walks for a minute at 0.02 m/s, and 12 is too stiff
    static constexpr float kpPerNm = 9.0f;
    static constexpr float kdPerNm = 0.25f;

    // s, how much the velocity target is smoothed. Each leg joint's MIT-mode velocity target is the
    // rate its commanded angle changes at: with a target of 0, the motor's kd term brakes every move,
    // and a 0.16 s swing lags 20-30 ms behind its plan and barely leaves the ground. The rate is
    // low-passed over this long, since the engine's commands can step (a phase change, the IK), and
    // capped at the joint's speed limit
    static constexpr float kVelocityFilter = 0.01f;

    // The head: fastest it turns to a target (rad/s), and the bob's nod (rad, at a step's full lift)
    // and roll (rad)
    static constexpr float kHeadSpeed = 4.0f;
    static constexpr float kBobNod = 0.10f;
    static constexpr float kBobRoll = 0.06f;

private:
    static constexpr int kLegJoints = DraxiKinematics::kLegJoints;

    void useOurKinematics();

    gait_core::GaitParams params_;
    gait_core::RobotDefinition robot_;
    DraxiKinematics left_, right_;   // before engine_: it points at them, so they must outlive it
    gait_core::GaitEngine engine_;

    std::array<float, kNumJoints> holdTargets_{};  // for the joints the engine doesn't drive
    std::array<float, kNumJoints> velocityTargets_{};  // rad/s, low-passed command rates (see kVelocityFilter)
    gait_core::Timestamp now_ = 0;                 // ns, advanced by dt each update

    // The head (see lookAt, setBob)
    void headCommands(const JointStates& states, const gait_core::GaitOutput& output, float dt, JointCommands& out);
    gait_core::Vec3 lookTarget_ = gait_core::Vec3::Zero();
    bool looking_ = false;
    bool bob_ = false;
    std::array<float, 4> headAngles_{};            // rad: neck yaw, pitch, nod, head roll as last commanded
    bool headStarted_ = false;
};

} // namespace draxi

#endif // DRAXI_GAIT_CONTROLLER_HPP
