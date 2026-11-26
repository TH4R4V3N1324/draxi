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
    // Masses: the body includes the neck and head (2.993 kg), the thigh includes the knee crank
    // and pushrod at zero knee angle. Whole robot 7.02 kg.
    robot.bodyMass = static_cast<Scalar>(2.9928);
    robot.centreOfMass = v(0.04405, -0.00015, 0.03356);

    auto leg = [&](bool left, JointId hipYaw, JointId hipRoll, JointId hipPitch, JointId knee,
                   JointId ankle) {
        const double side = left ? 1 : -1;  // +y for the left leg

        LegDefinition def{};
        def.mount = {0, v(0, side * 0.048, -0.062)};  // hip centre, in the body frame
        def.numJoints = 5;
        // The three hip axes meet at one point, so the first three origins are all zero
        def.joints[0] = revolute(v(0, 0, 0), v(0, 0, 1), hipYaw, RS05_SPEED);
        def.joints[1] = revolute(v(0, 0, 0), v(1, 0, 0), hipRoll, RS02_SPEED);
        def.joints[2] = revolute(v(0, 0, 0), v(0, 1, 0), hipPitch, RS02_SPEED);
        def.joints[3] = revolute(v(0, side * 0.044, -0.090), v(0, 1, 0), knee, RS02_SPEED);
        def.joints[4] = revolute(v(0.100, 0, 0), v(0, 1, 0), ankle, RS05_SPEED);
        def.foot = v(0.0272, side * 0.0007, -0.051);  // ankle to sole
        def.solver = gait_core::LegSolver::Chain;      // five joints: the generic solver

        // The link each joint turns: mass (kg) and centre of mass in that joint's frame (m)
        def.joints[0].mass = static_cast<Scalar>(0.5028);  // hip yaw bracket
        def.joints[0].centreOfMass = v(0.05818, 0, 0.00512);
        def.joints[1].mass = static_cast<Scalar>(0.5270);  // hip roll bracket
        def.joints[1].centreOfMass = v(-0.00034, side * 0.00342, -0.00005);
        def.joints[2].mass = static_cast<Scalar>(0.6375);  // thigh + knee crank + pushrod
        def.joints[2].centreOfMass = v(-0.00323, side * 0.06696, -0.01343);
        def.joints[3].mass = static_cast<Scalar>(0.2632);  // shin
        def.joints[3].centreOfMass = v(0.08833, side * -0.00028, 0.00008);
        def.joints[4].mass = static_cast<Scalar>(0.0844);  // foot
        def.joints[4].centreOfMass = v(0.01338, side * 0.00121, -0.02763);
                    
        const Scalar inner = static_cast<Scalar>(0.0262);  // m, the foot's inner corner (x forward, y left)
        const Scalar outer = static_cast<Scalar>(0.0283);  // m, the foot's outer corner

        def.soleMin = Vec2(Scalar(-0.055), left ? -inner : -outer);
        def.soleMax = Vec2(Scalar(0.055), left ? outer : inner);

        def.stanceOffset = v(0, side * 0.0447, 0);

        return def;
    };

    robot.legs[LEFT] = leg(true, JointId::HipYawL, JointId::HipRollL, JointId::HipPitchL,
                           JointId::KneeL, JointId::AnkleL);
    robot.legs[RIGHT] = leg(false, JointId::HipYawR, JointId::HipRollR, JointId::HipPitchR,
                            JointId::KneeR, JointId::AnkleR);

    robot.standReach    = Scalar(0.0238); 
    robot.standHeight   = Scalar(-0.161);
    robot.maxStride     = Scalar(0.03);

    robot.numGaits = 1;
    robot.gaits[0] = gait_core::GaitPattern::fromGroups("walk", {{LEFT}, {RIGHT}});  // left then right
    
    return robot;
}

}  // namespace draxi
