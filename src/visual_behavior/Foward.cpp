#include "visual_behavior/Foward.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/Twist.h"

#include "ros/ros.h"
#include <string>
#include <iostream>

namespace visual_behavior
{

Foward::Foward(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
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

    BT::Optional<std::string> turn_velocity = getInput<std::string>("turn_velocity");
    BT::Optional<std::string> foward_velocity = getInput<std::string>("foward_velocity");
    detected_ts_ = ros::Time::now();

    if (!turn_velocity) {
        throw BT::RuntimeError("missing required input [message]: ", turn_velocity.error() );
    } else if (!foward_velocity) {
        throw BT::RuntimeError("missing required input [message]: ", foward_velocity.error() );
    }

    cmd.linear.x = std::stod(foward_velocity.value());
    cmd.angular.z = std::stod(turn_velocity.value());

    while ((ros::Time::now() - detected_ts_).toSec() < WALKING_TIME) {
        pub_vel_.publish(cmd);
    }
    
    return BT::NodeStatus::SUCCESS;
}

}  // namespace visual_behavior
