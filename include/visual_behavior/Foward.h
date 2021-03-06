#ifndef VISUAL_BEHAVIOR_FOWARD_H
#define VISUAL_BEHAVIOR_FOWARD_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"

#include "geometry_msgs/Twist.h"

#include <string>

namespace visual_behavior
{

class Foward : public BT::ActionNodeBase
{
  public:
    explicit Foward(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick();
    void halt();

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("turn_velocity"), BT::InputPort<std::string>("foward_velocity")};
    }

  private:
    static constexpr double WALKING_TIME = 0.5;

    ros::Time detected_ts_;
    ros::NodeHandle n_;
    ros::Publisher pub_vel_;
};

}  // namespace visual_behavior

#endif  // VISUAL_BEHAVIOR_FOWARD_H