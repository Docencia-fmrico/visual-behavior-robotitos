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
  found_person_ = false;
  sub_darknet_ = n_.subscribe("/darknet_ros/bounding_boxes",1,&DetectPerson::DetectPersonCallBack,this);
}

void
DetectPerson::DetectPersonCallBack(const darknet_ros_msgs::BoundingBoxes::ConstPtr& boxes) {
  for (const auto & box : boxes->bounding_boxes) {
     if (box.Class == "person") {
        found_person_ =true;
     }
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
    setOutput("foward_velocity", "0.5" );
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace visual_behavior