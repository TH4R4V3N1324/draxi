#include "draxi/draxi.hpp"

#include <limits>

#include "draxi/joints.hpp"

namespace draxi {

namespace {

/**
 * @brief Helper to make a revolute joint definition
 * @param origin The joint's position relative to the previous joint (or the leg frame origin),
 *        at zero angles (m)
 * @param axis The joint's unit axis, in the leg frame at zero angles
 * @param joint Which joint in kJoints, for its limits
 * @param maxVelocity The fastest its motor may turn (rad/s); infinity for GaitParams::max_joint_speed
*/
JointDefinition revolute(const Vec3& origin, const Vec3& axis, JointId joint,
                         Scalar maxVelocity = std::numeric_limits<Scalar>::infinity()) {
    const JointInfo& info = kJoints[static_cast<size_t>(joint)];
    return {origin, axis, info.minRad, info.maxRad, maxVelocity};
}

/**
 * @brief Helper to make a Vec3 from doubles
 * @param x The x component
 * @param y The y component
 * @param z The z component
*/
Vec3 v(double x, double y, double z) {
    return Vec3(static_cast<Scalar>(x), static_cast<Scalar>(y), static_cast<Scalar>(z));
}

}  // namespace

RobotDefinition robot() {
    RobotDefinition robot{};

    robot.numLegs = NUM_LEGS;

    constexpr Scalar RS02_SPEED = 10.47198f;     // rad/s, RobStride 02 rated load speed
    constexpr Scalar RS05_SPEED = 10.47198f;     // rad/s, RobStride 05 rated load speed

    // Everything below is from cad/draxi.xml (bodies, joints and <inertial>s), left leg, with y
    // mirrored for the right. Each leg frame has yaw 0, so it lines up with the body frame (+x
    // forward, +y left, +z up) and the model's axes and offsets carry over unchanged. Limits come
    // from kJoints, which the sim checks against the model.
    //
    // Masses: the body is base_link alone; the neck and head are an appendage (below), so the engine
    // follows their weight as the head turns. The thigh includes the knee crank and pushrod at zero
    // knee angle. Whole robot 7.48 kg (urdf_v2).
    robot.bodyMass = static_cast<Scalar>(1.9598);
    robot.centreOfMass = v(-0.0196517, 0.000100607, 0.0161897);

    // The neck and head: yaw, pitch, nod and roll, from cad/draxi.xml. The engine doesn't move them
    // (GaitController does) but counts their weight wherever they are; a head carries nothing.
    gait_core::AppendageDefinition neck;
    neck.mount = {0, v(0.08, 0, 0.035)};  // the neck's yaw joint on the body
    neck.numJoints = 4;
    neck.joints[0] = revolute(v(0, 0, 0), v(0, 0, 1), JointId::NeckYaw);
    neck.joints[0].mass = static_cast<Scalar>(0.2249);
    neck.joints[0].centreOfMass = v(-2.80063e-06, -0.00168892, 0.0296704);
    neck.joints[1] = revolute(v(0, 0, 0.032), v(0, 1, 0), JointId::NeckPitch);
    neck.joints[1].mass = static_cast<Scalar>(0.0652);
    neck.joints[1].centreOfMass = v(0.03, 0.00171797, 0);
    neck.joints[2] = revolute(v(0.06, 0, 0), v(0, 1, 0), JointId::NeckNod);
    neck.joints[2].mass = static_cast<Scalar>(0.2323);
    neck.joints[2].centreOfMass = v(0.00393668, -0.00172834, 9.15234e-05);
    neck.joints[3] = revolute(v(0.0415, 0, 0), v(1, 0, 0), JointId::HeadRoll);
    neck.joints[3].mass = static_cast<Scalar>(0.5106);
    neck.joints[3].centreOfMass = v(0.0330524, 4.1628e-05, -1.23227e-06);
    neck.tip = v(0.08, 0, 0);   // the face
    neck.maxPayload = 0;
    robot.numAppendages = 1;
    robot.appendages[NECK] = neck;

    auto leg = [&](bool left, JointId hipYaw, JointId hipRoll, JointId hipPitch, JointId knee,
                   JointId ankle, JointId ankleRoll) {
        const double side = left ? 1 : -1;  // +y for the left leg

        LegDefinition def{};
        def.mount = {0, v(0, side * 0.048, -0.062)};  // hip centre, in the body frame
        def.numJoints = 6;
        // The three hip axes meet at one point, so the first three origins are all zero
        def.joints[0] = revolute(v(0, 0, 0), v(0, 0, 1), hipYaw, RS05_SPEED);
        def.joints[1] = revolute(v(0, 0, 0), v(1, 0, 0), hipRoll, RS02_SPEED);
        def.joints[2] = revolute(v(0, 0, 0), v(0, 1, 0), hipPitch, RS02_SPEED);
        def.joints[3] = revolute(v(0, side * 0.044, -0.090), v(0, 1, 0), knee, RS02_SPEED);
        def.joints[4] = revolute(v(0.100, side * 0.002, 0), v(0, 1, 0), ankle, RS05_SPEED);
        // Ankle roll, after the pitch at the same point: it keeps the sole flat when the hip rolls
        // the leg to move the weight over a foot
        def.joints[5] = revolute(v(0, 0, 0), v(1, 0, 0), ankleRoll, RS05_SPEED);
        def.foot = v(0.0375, 0, -0.051);  // ankle to sole
        def.solver = gait_core::LegSolver::Chain;      // DraxiKinematics replaces it (GaitController)

        // The link each joint turns: mass (kg) and centre of mass in that joint's frame (m)
        def.joints[0].mass = static_cast<Scalar>(0.5028);  // hip yaw bracket
        def.joints[0].centreOfMass = v(0.05818, 0, 0.00512);
        def.joints[1].mass = static_cast<Scalar>(0.5270);  // hip roll bracket
        def.joints[1].centreOfMass = v(-0.00034, side * 0.00342, -0.00005);
        def.joints[2].mass = static_cast<Scalar>(0.6375);  // thigh + knee crank + pushrod
        def.joints[2].centreOfMass = v(-0.00323, side * 0.06696, -0.01343);
        def.joints[3].mass = static_cast<Scalar>(0.2632);  // shin
        def.joints[3].centreOfMass = v(0.08833, side * -0.00028, 0.00008);
        def.joints[4].mass = static_cast<Scalar>(0.0498);  // ankle (between the pitch and roll axes)
        def.joints[4].centreOfMass = v(0.01678, side * 0.00134, 0);
        def.joints[5].mass = static_cast<Scalar>(0.2617);  // foot + ankle roll motor (the ankle roll turns it)
        def.joints[5].centreOfMass = v(0.05834, side * -0.00068, -0.00830);

        // The sole's flat face around the sole point, from the foot's collision meshes (m, x forward)
        const Scalar back = static_cast<Scalar>(0.0653);
        const Scalar front = static_cast<Scalar>(0.0505);
        const Scalar inner = static_cast<Scalar>(0.0281);
        const Scalar outer = static_cast<Scalar>(0.0290);

        def.soleMin = Vec2(-back, left ? -inner : -outer);
        def.soleMax = Vec2(front, left ? outer : inner);

        // Each foot 38 mm out from the middle, 8 mm in from straight under its hip (the knee's 44 mm
        // and the ankle's 2 mm): less to sway across each step (sim, 2026-11-27). 34-38 mm
        // walk alike; 40 mm and wider fall at 0.16 m/s. Leaves 20 mm between the soles.
        def.stanceOffset = v(0, side * 0.038, 0);

        return def;
    };

    robot.legs[LEFT] = leg(true, JointId::HipYawL, JointId::HipRollL, JointId::HipPitchL,
                           JointId::KneeL, JointId::AnklePitchL, JointId::AnkleRollL);
    robot.legs[RIGHT] = leg(false, JointId::HipYawR, JointId::HipRollR, JointId::HipPitchR,
                            JointId::KneeR, JointId::AnklePitchR, JointId::AnkleRollR);

    // The sole point 33.5 mm ahead of the hip: with the urdf_v2 feet (sim, 2026-11-27) this
    // walks 0.04-0.16 m/s at 100-104% of the command; 31 mm falls behind at 0.16 m/s, and 36 mm
    // sways and tilts more
    robot.standReach    = Scalar(0.0335);
    robot.standHeight   = Scalar(-0.161);
    robot.maxStride     = Scalar(0.03);

    robot.numGaits = 1;
    // Left swings, both feet down while the weight moves across, right swings, both down again
    robot.gaits[0] = gait_core::GaitPattern::fromGroups("walk", {{LEFT}, {}, {RIGHT}, {}});
    
    return robot;
}

}  // namespace draxi
