#include "draxi/draxiKinematics.hpp"

#include <algorithm>
#include <cmath>

namespace draxi {

namespace {

// An angle wrapped to (-pi, pi]
gait_core::Scalar wrap(gait_core::Scalar a) { return std::atan2(std::sin(a), std::cos(a)); }

}  // namespace

// Lengths from the same LegDefinition as robot(), so the two can't disagree
DraxiKinematics::DraxiKinematics(const gait_core::LegDefinition& leg)
    : thighLength_(-leg.joints[3].origin.z()), shinLength_(leg.joints[4].origin.x()),
      kneeOutboard_(leg.joints[3].origin.y()), soleOutboard_(leg.joints[3].origin.y() + leg.foot.y()),
      footForward_(leg.foot.x()), footDown_(leg.foot.z()) {
    for (int i = 0; i < 5; ++i) { lower_[i] = leg.joints[i].lower; upper_[i] = leg.joints[i].upper; }
}

// A point in the leg's plane (x forward, y out along the pitch axis, z down the leg),
// rolled about x then yawed about z into the leg frame
gait_core::Vec3 DraxiKinematics::place(const JointVec& q, Scalar x, Scalar y, Scalar z) const {
    const Scalar c0 = std::cos(q[0]), s0 = std::sin(q[0]), c1 = std::cos(q[1]), s1 = std::sin(q[1]);
    const Scalar yr = c1 * y - s1 * z, zr = s1 * y + c1 * z;
    return Vec3(c0 * x - s0 * yr, s0 * x + c0 * yr, zr);
}

gait_core::Vec3 DraxiKinematics::fk(const JointVec& q) const {
    const Scalar knee = q[2] + q[3], foot = q[2] + q[3] + q[4];  // shin and foot pitch
    const Scalar x = -thighLength_ * std::sin(q[2]) + shinLength_ * std::cos(knee)
                   + footForward_ * std::cos(foot) + footDown_ * std::sin(foot);
    const Scalar z = -thighLength_ * std::cos(q[2]) - shinLength_ * std::sin(knee)
                   - footForward_ * std::sin(foot) + footDown_ * std::cos(foot);
    return place(q, x, soleOutboard_, z);
}

bool DraxiKinematics::ik(const Vec3& target, const JointVec& /*seed*/, JointVec& q) const {
    bool ok = true;
    // 1. Undo the yaw: the foot's heading is chosen, not solved
    const Scalar c0 = std::cos(footYaw_), s0 = std::sin(footYaw_);
    const Scalar x = c0 * target.x() + s0 * target.y();
    const Scalar y = -s0 * target.x() + c0 * target.y();
    const Scalar z = target.z();
    // 2. Roll: the foot is soleOutboard_ out along the pitch axis, so its depth in the plane is the rest
    Scalar depth2 = y * y + z * z - soleOutboard_ * soleOutboard_;
    if (depth2 < 0) { depth2 = 0; ok = false; }
    const Scalar vz = -std::sqrt(depth2);  // assumes the foot is below the hip
    const Scalar roll = std::atan2(z, y) - std::atan2(vz, soleOutboard_);
    // 3. Take the sole offset off at the wanted foot pitch: a 2-link thigh/shin problem
    const Scalar wx = x - (footForward_ * std::cos(footPitch_) + footDown_ * std::sin(footPitch_));
    const Scalar wz = vz - (-footForward_ * std::sin(footPitch_) + footDown_ * std::cos(footPitch_));
    Scalar s3 = (wx * wx + wz * wz - thighLength_ * thighLength_ - shinLength_ * shinLength_) / (2 * thighLength_ * shinLength_);
    if (s3 < -1 || s3 > 1) { s3 = std::clamp(s3, Scalar(-1), Scalar(1)); ok = false; }
    const Scalar knee = std::asin(s3);
    const Scalar hipPitch = wrap(std::atan2(-thighLength_ - shinLength_ * s3, shinLength_ * std::cos(knee)) - std::atan2(wz, wx));
    // 4. The ankle makes up the foot's pitch
    const Scalar ankle = wrap(footPitch_ - hipPitch - knee);
    q.resize(5);
    q << footYaw_, roll, hipPitch, knee, ankle;
    for (int i = 0; i < 5; ++i) ok = ok && q[i] >= lower_[i] && q[i] <= upper_[i];
    return ok;
}

// Exact: each column is the joint's axis crossed with the lever from the joint to the sole
gait_core::LegJacobian DraxiKinematics::jacobian(const JointVec& q) const {
    const Vec3 p = fk(q);
    const Scalar c0 = std::cos(q[0]), s0 = std::sin(q[0]), c1 = std::cos(q[1]), s1 = std::sin(q[1]);
    const Vec3 yawAxis(0, 0, 1), rollAxis(c0, s0, 0), pitchAxis(-s0 * c1, c0 * c1, s1);
    const Scalar kx = -thighLength_ * std::sin(q[2]), kz = -thighLength_ * std::cos(q[2]);
    const Vec3 knee = place(q, kx, kneeOutboard_, kz);
    const Vec3 ankle = place(q, kx + shinLength_ * std::cos(q[2] + q[3]), kneeOutboard_, kz - shinLength_ * std::sin(q[2] + q[3]));
    LegJacobian J(3, 5);
    J.col(0) = yawAxis.cross(p);
    J.col(1) = rollAxis.cross(p);
    J.col(2) = pitchAxis.cross(p);
    J.col(3) = pitchAxis.cross(p - knee);
    J.col(4) = pitchAxis.cross(p - ankle);
    return J;
}

} // namespace draxi