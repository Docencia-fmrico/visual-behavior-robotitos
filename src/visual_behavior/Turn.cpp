#include "visual_behavior/Turn.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/Twist.h"

#include "ros/ros.h"
#include <string>

namespace visual_behavior
{

Turn::Turn(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, {})
{
    pub_vel_ = n_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);
}

void
Turn::halt()
{
  ROS_INFO("Turn halt");
}
 
BT::NodeStatus
Turn::tick()
{
    geometry_msgs::Twist cmd;

    if (status() == BT::NodeStatus::IDLE)
    {
        ROS_INFO("Turning");
    }

    BT::Optional<std::string>  turn_velocity = getInput<std::string>("turn_velocity");
    BT::Optional<std::string>  turn_direction = getInput<std::string>("turn_direction");

    if (!turn_direction)
    {
        throw BT::RuntimeError("missing required input [message]: ", turn_direction.error() );
    } else if (!turn_velocity) {
        throw BT::RuntimeError("missing required input [message]: ", turn_velocity.error() );
    }
    ROS_INFO("Turning obteniendo valores");
    if (turn_direction.value() == "right") {
        cmd.linear.x = 0.0;
        cmd.angular.z = -std::stod(turn_velocity.value());
    } else {
        cmd.linear.x = 0.0;
        cmd.angular.z = std::stod(turn_velocity.value());
    }

    pub_vel_.publish(cmd);

    return BT::NodeStatus::RUNNING;
}

}  // namespace visual_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<visual_behavior::Turn>("Turn");
}