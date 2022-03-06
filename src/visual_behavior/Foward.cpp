#include "visual_behavior/Foward.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/Twist.h"

#include "ros/ros.h"

#include <string>

namespace visual_behavior
{

Foward::Foward(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, {})
{
    pub_vel_ = n_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);
}

void
Foward::halt()
{
  ROS_INFO("Foward halt");
}

BT::NodeStatus
Foward::tick()
{
    geometry_msgs::Twist cmd;

    if (status() == BT::NodeStatus::IDLE)
    {
        ROS_INFO("Foward");
    }

    BT::Optional<std::string> foward = getInput<std::string>("foward_direction");

    if (foward.value() == "back") {
        cmd.linear.x = -0.1;
        cmd.angular.z = 0.0;
    } else {
        cmd.linear.x = 0.1;
        cmd.angular.z = 0.0;
    }

    pub_vel_.publish(cmd);
    
    return BT::NodeStatus::SUCCESS;
}

}  // namespace visual_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<visual_behavior::Foward>("Foward");
}