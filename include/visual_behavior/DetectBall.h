#ifndef VISUAL_BEHAVIOR_DETECTBALL_H
#define VISUAL_BEHAVIOR_DETECTBALL_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"


#include <string>

namespace visual_behavior
{

class DetectBall : public BT::ActionNodeBase
{
  public:
    explicit DetectBall(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick();
    void halt();

    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<std::string>("turn") };
    }

  private:

    ros::NodeHandle n_;
    ros::Subscriber sub_darknet_;
};

}  // namespace visual_behavior

#endif  // VISUAL_BEHAVIOR_DETECTBALL_H