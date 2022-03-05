#include "visual_behavior/Foward.h"

#include "geometry_msgs/Twist.h"

#include "ros/ros.h"

namespace visual_behavior
{

Foward::Foward(const std::string& name)
: BT::ActionNodeBase(name, {})
{
    pub_vel_ = n_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);
}
 
BT::NodeStatus
Foward::tick()
{
    geometry_msgs::Twist cmd;

    if (status() == BT::NodeStatus::IDLE)
    {
        ROS_INFO("Foward");
    }

    std::string foward = getInput<std::string>("foward_direction").value();

    if (foward == "back") {
        cmd.linear.x = -0.1;
        cmd.angular.z = 0.0;
    } else {
        cmd.linear.x = 0.1;
        cmd.angular.z = 0.0;
    }

    pub_vel_.publish(cmd);

    if ((ros::Time::now()-turn_ts_).toSec() < BACKING_TIME )
    {
        return BT::NodeStatus::RUNNING;
    } else {
        return BT::NodeStatus::SUCCESS;
    }
}

}  // namespace visual_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behavior_trees::Foward>("Foward");
}