#include "visual_behavior/DetectPerson.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <darknet_ros_msgs/BoundingBoxes.h>

#include "ros/ros.h"
#include <string>

namespace visual_behavior
{

DetectPerson::DetectPerson(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, {})
{
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> sub_darknet_(n_, "/darknet_ros/bounding_boxes", 1);
}

void
DetectPerson::DetectPersonCallBack(const darknet_ros_msgs::BoundingBoxes::ConstPtr& boxes) {

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

  std::string Person = "false";
  setOutput("turn", Person );

  if (Person == "true") {
    setOutput("turn_direction", "right" );
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace visual_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<visual_behavior::DetectPerson>("DetectPerson");
}