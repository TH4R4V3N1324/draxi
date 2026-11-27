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

// The robot's appendages in gait-core (RobotDefinition::appendages): the neck and head
enum appendage : uint8_t {
    NECK,
    NUM_APPENDAGES
};

RobotDefinition robot();

} // namespace draxi

#endif // DRAXI_DEFINITION_HPP