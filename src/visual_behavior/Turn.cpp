#include "visual_behavior/Turn.h"

#include "geometry_msgs/Twist.h"

#include "ros/ros.h"

namespace visual_behavior
{

Turn::Turn(const std::string& name)
: BT::ActionNodeBase(name, {})
{
    pub_vel_ = n_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);
}
  
 
BT::NodeStatus
Turn::tick()
{
    geometry_msgs::Twist cmd;

    if (status() == BT::NodeStatus::IDLE)
    {
        ROS_INFO("Turning");
    }

    std::string turn = getInput<std::string>("turn_direction").value();

    if (turn == "right") {
        cmd.linear.x = 0.0;
        cmd.angular.z = -0.4;
    } else {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.4;
    }

    pub_vel_.publish(cmd);

    if ((ros::Time::now()-turn_ts_).toSec() < TURNING_TIME )
    {
        return BT::NodeStatus::RUNNING;
    } else {
        return BT::NodeStatus::SUCCESS;
    }
}

}  // namespace visual_behavior