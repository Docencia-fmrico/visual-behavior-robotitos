#ifndef VISUAL_BEHAVIOR_DETECTPERSON_H
#define VISUAL_BEHAVIOR_DETECTPERSON_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <message_filters/subscriber.h>

#include <string>

namespace visual_behavior
{

class DetectPerson : public BT::ActionNodeBase
{
  public:
    explicit DetectPerson(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick();
    void halt();
    void DetectPersonCallBack(const darknet_ros_msgs::BoundingBoxes::ConstPtr& boxes);

    static BT::PortsList providedPorts()
    {
        // This action has a single input port called "message"
        // Any port must have a name. The type is optional.
        return { BT::OutputPort<std::string>("foward_direction") };
    }

  private:

    ros::NodeHandle n_;
    ros::Subscriber sub_darknet_;
};

}  // namespace visual_behavior

#endif  // VISUAL_BEHAVIOR_DETECTPERSON_H