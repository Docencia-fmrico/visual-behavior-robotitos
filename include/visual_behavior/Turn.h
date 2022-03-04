#ifndef VISUAL_BEHAVIOR_TURN_H
#define VISUAL_BEHAVIOR_TURN_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"

#include "geometry_msgs/Twist.h"

#include <string>

namespace visual_behavior
{

class Turn : public BT::ActionNodeBase
{
  public:
    explicit Turn(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick();


    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("turn")};
    }

  private:
    static constexpr double TURNING_TIME = 3.0;

    ros::NodeHandle n_;
    ros::Publisher pub_vel_;
};

}  // namespace visual_behavior

#endif  // VISUAL_BEHAVIOR_TURN_H