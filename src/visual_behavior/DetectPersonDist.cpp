#include "visual_behavior/DetectPersonDist.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <darknet_ros_msgs/BoundingBoxes.h>

#include "ros/ros.h"
#include <string>

namespace visual_behavior
{

DetectPersonDist::DetectPersonDist(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
  found_person_ = false;
  sub_darknet_ = n_.subscribe("/darknet_ros/bounding_boxes",1,&DetectPersonDist::DetectPersonDistCallBack,this);
}

void
DetectPersonDist::DetectPersonDistCallBack(const darknet_ros_msgs::BoundingBoxes::ConstPtr& boxes) {
  for (const auto & box : boxes->bounding_boxes) {
     if (box.Class == "person") {
        found_person_ =true;
     }
  }
}

void
DetectPersonDist::halt()
{
  ROS_INFO("DetectPersonDist halt");
}

BT::NodeStatus
DetectPersonDist::tick()
{
  if (status() == BT::NodeStatus::IDLE)
  {
    ROS_INFO("Loking for a person and return a distance");
  }

  if (found_person_ == true) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace visual_behavior