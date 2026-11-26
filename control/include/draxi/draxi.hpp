#ifndef DRAXI_DEFINITION_HPP
#define DRAXI_DEFINITION_HPP

#include <gait_core/robot/robot_definition.hpp>
#include <gait_core/types.hpp>
#include "draxi/draxiKinematics.hpp"

namespace draxi {

using Scalar = gait_core::Scalar;
using Vec3 = gait_core::Vec3;
using Vec2 = gait_core::Vec2;
using JointDefinition = gait_core::JointDefinition;
using LegDefinition = gait_core::LegDefinition;
using RobotDefinition = gait_core::RobotDefinition;

enum leg : uint8_t {
    LEFT,
    RIGHT,
    NUM_LEGS
};

RobotDefinition robot();

} // namespace draxi

#endif // DRAXI_DEFINITION_HPP