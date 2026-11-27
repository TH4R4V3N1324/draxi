#ifndef DRAXI_KINEMATICS_HPP
#define DRAXI_KINEMATICS_HPP

#include <gait_core/kinematics/leg_kinematics.hpp>
#include <gait_core/robot/leg_definition.hpp>
#include <gait_core/types.hpp>
#include <array>

namespace draxi {

// Draxi's leg, solved analytically: hip yaw, roll and pitch (their axes meet at the hip), knee, ankle
// pitch and ankle roll (at the same point). The sole is put where it's asked and turned the way it's
// asked: its heading by the hip yaw, its pitch by the ankle pitch, its roll by the ankle roll (making
// up for the hip's roll), so it stays flat on the ground however the hips move the body over it.
class DraxiKinematics : public gait_core::LegKinematics {
public:
    using JointVec = gait_core::JointVec;
    using Vec3 = gait_core::Vec3;
    using Mat3 = gait_core::Mat3;
    using Scalar = gait_core::Scalar;
    using LegJacobian = gait_core::LegJacobian;

    static constexpr int kLegJoints = 6;  // hip yaw, hip roll, hip pitch, knee, ankle, ankle roll

    explicit DraxiKinematics(const gait_core::LegDefinition& leg);

    int numJoints() const override { return kLegJoints; }
    // The sole flat and turned as setFootOrientation says (straight ahead by default)
    bool ik(const Vec3& target, const JointVec& seed, JointVec& q) const override;
    // The sole turned as gait-core asks: flat on the ground, facing the way it was put down
    bool ikWithSole(const Vec3& target, const Mat3& sole, const JointVec& seed, JointVec& q) const override;
    Vec3 fk(const JointVec& q) const override;
    LegJacobian jacobian(const JointVec& q) const override;
    // For ik: the sole's heading (= hip yaw) and pitch relative to the leg, 0, 0 for flat and straight
    void setFootOrientation(Scalar yaw, Scalar pitch) { footYaw_ = yaw; footPitch_ = pitch; }

private:
    // The sole at target, turned by yaw, then roll, then pitch (Z X Y) in the leg frame
    bool solveFoot(const Vec3& target, Scalar yaw, Scalar pitch, Scalar roll, JointVec& q) const;
    // A point in the leg's plane, rolled then yawed into the leg frame
    Vec3 place(const JointVec& q, Scalar x, Scalar y, Scalar z) const;
    Vec3 anklePoint(const JointVec& q) const;
    Mat3 footTurn(const JointVec& q) const;

    Scalar thighLength_;   // hip pitch axis to knee axis: -joints[3].origin.z    (0.090)
    Scalar shinLength_;    // knee to ankle:                joints[4].origin.x    (0.100)
    Scalar kneeOutboard_;  // hip to ankle, along the pitch axis: joints[3].origin.y + joints[4].origin.y (+-0.046)
    Scalar footForward_;   // ankle to sole: foot.x  (0.0375)
    Scalar footOutboard_;  // ankle to sole: foot.y  (0)
    Scalar footDown_;      // ankle to sole: foot.z  (-0.051, negative = below)
    std::array<Scalar, kLegJoints> lower_{}, upper_{};
    Scalar footYaw_ = 0, footPitch_ = 0;
};

}  // namespace draxi

#endif  // DRAXI_KINEMATICS_HPP
