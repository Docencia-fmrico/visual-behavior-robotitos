#include "visual_behavior/DetectPerson.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <darknet_ros_msgs/ObjectCount.h>

#include "ros/ros.h"
#include <string>

namespace visual_behavior
{

DetectPerson::DetectPerson(const std::string& name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  found_person_ = true;
  sub_darknet_ = n_.subscribe("/darknet_ros/found_object", 1, &DetectPerson::DetectPersonCallBack,this);
  sub_darknet_ = n_.subscribe("/darknet_ros/found_object", 1, &DetectPerson::DetectPersonCallBack,this);
}

void
DetectPerson::DetectPersonCallBack(const darknet_ros_msgs::ObjectCount::ConstPtr& boxes) {
  ROS_INFO(" callback detectperson");

  if (boxes->count >= 1) {
    found_person_ = true;
  } else {
    found_person_ = false;

  }
}

void
DetectPerson::halt()
{
  ROS_INFO("DetectPerson halt");
}

BT::NodeStatus
DetectPerson::tick()
{
  if (status() == BT::NodeStatus::IDLE)
  {
    ROS_INFO("Loking for a person");
  }

  if (found_person_ == true) {
    return BT::NodeStatus::SUCCESS;
  } else {
    setOutput("turn_direction", "rigth" );
    setOutput("turn_velocity", "0.5" );
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace visual_behavior