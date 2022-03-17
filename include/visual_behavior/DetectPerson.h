#ifndef VISUAL_BEHAVIOR_DETECTPERSON_H
#define VISUAL_BEHAVIOR_DETECTPERSON_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"

#include <darknet_ros_msgs/ObjectCount.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include <string>

namespace visual_behavior
{

class DetectPerson : public BT::ConditionNode
{
  public:
    explicit DetectPerson(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick();
    void DetectPersonCallBack(const darknet_ros_msgs::BoundingBoxes::ConstPtr& boxes);
    void CounterCallBack(const darknet_ros_msgs::ObjectCount::ConstPtr& counter);

    static BT::PortsList providedPorts()
    {
        return{ BT::OutputPort<std::string>("turn_velocity"), BT::OutputPort<std::string>("counter")};
    }

  private:
    bool found_person_;
    int contador = 0;
    ros::NodeHandle n_;
    ros::Subscriber sub_counter_;
    ros::Subscriber sub_darknet_;
};

}  // namespace visual_behavior

#endif  // VISUAL_BEHAVIOR_DETECTPERSON_H