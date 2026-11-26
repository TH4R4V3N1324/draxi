#ifndef DRAXI_KINEMATICS_HPP
#define DRAXI_KINEMATICS_HPP

#include <gait_core/kinematics/leg_kinematics.hpp>
#include <gait_core/robot/leg_definition.hpp>
#include <gait_core/types.hpp>
#include <array>

namespace draxi {

class DraxiKinematics : public gait_core::LegKinematics {
public:
    using JointVec = gait_core::JointVec;
    using Vec3 = gait_core::Vec3;
    using Scalar = gait_core::Scalar;
    using LegJacobian = gait_core::LegJacobian;

    static constexpr int kLegJoints = 5;  // hip yaw, hip roll, hip pitch, knee, ankle

    explicit DraxiKinematics(const gait_core::LegDefinition& leg);
    // Foot heading (= hip yaw) and pitch relative to the leg: 0, 0 for flat and straight

    int numJoints() const override { return kLegJoints; }
    bool ik(const Vec3& target, const JointVec& seed, JointVec& q) const override;
    Vec3 fk(const JointVec& q) const override;
    LegJacobian jacobian(const JointVec& q) const override;
    void setFootOrientation(Scalar yaw, Scalar pitch) { footYaw_ = yaw; footPitch_ = pitch; }

private:
    // A point in the leg's plane, rolled then yawed into the leg frame
    Vec3 place(const JointVec& q, Scalar x, Scalar y, Scalar z) const;

    Scalar thighLength_;   // hip pitch axis to knee axis: -joints[3].origin.z    (0.090)
    Scalar shinLength_;    // knee to ankle:                joints[4].origin.x    (0.100)
    Scalar kneeOutboard_;  // hip to knee, along the pitch axis: joints[3].origin.y   (±0.044)
    Scalar soleOutboard_;  // hip to sole, along the pitch axis: kneeOutboard + foot.y (±0.0447)
    Scalar footForward_;   // ankle to sole: foot.x  (0.0272)
    Scalar footDown_;      // ankle to sole: foot.z  (-0.051, negative = below)
    std::array<Scalar, kLegJoints> lower_{}, upper_{};
    Scalar footYaw_ = 0, footPitch_ = 0;

};

}  // namespace draxi

#endif  // DRAXI_KINEMATICS_HPP