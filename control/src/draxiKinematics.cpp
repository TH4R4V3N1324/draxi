#include "draxi/draxiKinematics.hpp"

#include <algorithm>
#include <cmath>

namespace draxi {

namespace {

// An angle wrapped to (-pi, pi]
gait_core::Scalar wrap(gait_core::Scalar a) { return std::atan2(std::sin(a), std::cos(a)); }

}  // namespace

// Lengths from the same LegDefinition as robot(), so the two can't disagree. Assumes the ankle roll's
// origin is the ankle pitch's (joints[5].origin zero). The shin's sideways step to the ankle is along
// the pitch axis, so it adds to the knee's and the ankle sits kneeOutboard_ out from the hip.
DraxiKinematics::DraxiKinematics(const gait_core::LegDefinition& leg)
    : thighLength_(-leg.joints[3].origin.z()), shinLength_(leg.joints[4].origin.x()),
      kneeOutboard_(leg.joints[3].origin.y() + leg.joints[4].origin.y()), footForward_(leg.foot.x()), footOutboard_(leg.foot.y()),
      footDown_(leg.foot.z()) {
    for (int i = 0; i < kLegJoints; ++i) { lower_[i] = leg.joints[i].lower; upper_[i] = leg.joints[i].upper; }
}

// A point in the leg's plane (x forward, y out along the pitch axis, z up), rolled about x then
// yawed about z into the leg frame
gait_core::Vec3 DraxiKinematics::place(const JointVec& q, Scalar x, Scalar y, Scalar z) const {
    const Scalar c0 = std::cos(q[0]), s0 = std::sin(q[0]), c1 = std::cos(q[1]), s1 = std::sin(q[1]);
    const Scalar yr = c1 * y - s1 * z, zr = s1 * y + c1 * z;
    return Vec3(c0 * x - s0 * yr, s0 * x + c0 * yr, zr);
}

gait_core::Vec3 DraxiKinematics::anklePoint(const JointVec& q) const {
    const Scalar knee = q[2] + q[3];
    return place(q, -thighLength_ * std::sin(q[2]) + shinLength_ * std::cos(knee), kneeOutboard_,
                 -thighLength_ * std::cos(q[2]) - shinLength_ * std::sin(knee));
}

// How the foot is turned: hip yaw, hip roll, the three pitches together, ankle roll
gait_core::Mat3 DraxiKinematics::footTurn(const JointVec& q) const {
    using gait_core::AngleAxis;
    return (AngleAxis(q[0], Vec3::UnitZ()) * AngleAxis(q[1], Vec3::UnitX()) *
            AngleAxis(q[2] + q[3] + q[4], Vec3::UnitY()) * AngleAxis(q[5], Vec3::UnitX())).toRotationMatrix();
}

gait_core::Vec3 DraxiKinematics::fk(const JointVec& q) const {
    return anklePoint(q) + footTurn(q) * Vec3(footForward_, footOutboard_, footDown_);
}

bool DraxiKinematics::ik(const Vec3& target, const JointVec& /*seed*/, JointVec& q) const {
    return solveFoot(target, footYaw_, footPitch_, 0, q);
}

// The sole's turn split as solveFoot builds it: yaw, then roll, then pitch (Z X Y)
bool DraxiKinematics::ikWithSole(const Vec3& target, const Mat3& sole, const JointVec& /*seed*/, JointVec& q) const {
    const Scalar yaw = std::atan2(sole(1, 0), sole(0, 0));
    const Mat3 rest = gait_core::AngleAxis(-yaw, Vec3::UnitZ()).toRotationMatrix() * sole;   // Rx(roll) Ry(pitch)
    return solveFoot(target, yaw, std::atan2(rest(0, 2), rest(0, 0)), std::atan2(rest(2, 1), rest(1, 1)), q);
}

bool DraxiKinematics::solveFoot(const Vec3& target, Scalar footYaw, Scalar footPitch, Scalar footRoll, JointVec& q) const {
    using gait_core::AngleAxis;
    bool ok = true;
    // 1. The ankle: the sole less the foot's offset, turned as the sole is to be
    const Mat3 sole = (AngleAxis(footYaw, Vec3::UnitZ()) * AngleAxis(footRoll, Vec3::UnitX()) *
                       AngleAxis(footPitch, Vec3::UnitY())).toRotationMatrix();
    const Vec3 ankle = target - sole * Vec3(footForward_, footOutboard_, footDown_);
    // 2. Undo the yaw: the foot's heading is chosen, not solved
    const Scalar c0 = std::cos(footYaw), s0 = std::sin(footYaw);
    const Scalar x = c0 * ankle.x() + s0 * ankle.y();
    const Scalar y = -s0 * ankle.x() + c0 * ankle.y();
    const Scalar z = ankle.z();
    // 3. Roll: the ankle is kneeOutboard_ out along the pitch axis, so its depth in the plane is the rest
    Scalar depth2 = y * y + z * z - kneeOutboard_ * kneeOutboard_;
    if (depth2 < 0) { depth2 = 0; ok = false; }
    const Scalar vz = -std::sqrt(depth2);  // assumes the foot is below the hip
    const Scalar roll = std::atan2(z, y) - std::atan2(vz, kneeOutboard_);
    // 4. Thigh and shin to the ankle: a 2-link problem in the plane
    Scalar s3 = (x * x + vz * vz - thighLength_ * thighLength_ - shinLength_ * shinLength_) / (2 * thighLength_ * shinLength_);
    if (s3 < -1 || s3 > 1) { s3 = std::clamp(s3, Scalar(-1), Scalar(1)); ok = false; }
    const Scalar knee = std::asin(s3);
    const Scalar hipPitch = wrap(std::atan2(-thighLength_ - shinLength_ * s3, shinLength_ * std::cos(knee)) - std::atan2(vz, x));
    // 5. The ankle pitch makes up the foot's pitch; the ankle roll undoes the hip's roll, to the sole's
    q.resize(kLegJoints);
    q << footYaw, wrap(roll), hipPitch, knee, wrap(footPitch - hipPitch - knee), wrap(footRoll - roll);
    for (int i = 0; i < kLegJoints; ++i) ok = ok && q[i] >= lower_[i] && q[i] <= upper_[i];
    return ok;
}

// Exact: each column is the joint's axis crossed with the lever from the joint to the sole
gait_core::LegJacobian DraxiKinematics::jacobian(const JointVec& q) const {
    const Vec3 p = fk(q);
    const Scalar c0 = std::cos(q[0]), s0 = std::sin(q[0]), c1 = std::cos(q[1]), s1 = std::sin(q[1]);
    const Vec3 yawAxis(0, 0, 1), rollAxis(c0, s0, 0), pitchAxis(-s0 * c1, c0 * c1, s1);
    const Vec3 knee = place(q, -thighLength_ * std::sin(q[2]), kneeOutboard_, -thighLength_ * std::cos(q[2]));
    const Vec3 ankle = anklePoint(q);
    // The ankle roll's axis is the foot's own forward axis (it turns about itself)
    const Vec3 ankleRollAxis = footTurn(q) * Vec3::UnitX();
    LegJacobian J(3, kLegJoints);
    J.col(0) = yawAxis.cross(p);
    J.col(1) = rollAxis.cross(p);
    J.col(2) = pitchAxis.cross(p);
    J.col(3) = pitchAxis.cross(p - knee);
    J.col(4) = pitchAxis.cross(p - ankle);
    J.col(5) = ankleRollAxis.cross(p - ankle);
    return J;
}

} // namespace draxi
