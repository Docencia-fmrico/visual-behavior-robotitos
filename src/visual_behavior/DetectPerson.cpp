#include "visual_behavior/DetectPerson.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <darknet_ros_msgs/ObjectCount.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include "ros/ros.h"
#include <string>

namespace visual_behavior
{

DetectPerson::DetectPerson(const std::string& name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config),found_person_(false)
{
  sub_counter_ = n_.subscribe("/darknet_ros/found_object", 1, &DetectPerson::CounterCallBack,this);
  sub_darknet_ = n_.subscribe("darknet_ros/bounding_boxes", 1, &DetectPerson::DetectPersonCallBack,this);
}

void
DetectPerson::CounterCallBack(const darknet_ros_msgs::ObjectCount::ConstPtr& counter) {
  ROS_INFO(" callback counter");

  if (counter->count >= 1) {
    found_person_ = true;
  } else {
    found_person_ = false;
  }
}

void
DetectPerson::DetectPersonCallBack(const darknet_ros_msgs::BoundingBoxesConstPtr& boxes){
  ROS_INFO(" callback detectperson");
  for (const auto & box : boxes->bounding_boxes) {
    if (box.Class =="person") {
      found_person_ = true;
    } else {
      found_person_ = false;
    }
  }
}

BT::NodeStatus
DetectPerson::tick()
{
  if (status() == BT::NodeStatus::IDLE)
  {
    ROS_INFO("Loking for a person");
  }

  contador++;
  if (found_person_) {
    setOutput("counter", "0");
    contador = 0;
    return BT::NodeStatus::SUCCESS;
  } else {
    setOutput("turn_direction", "rigth" );
    setOutput("turn_velocity", "-0.5" );
    setOutput("counter", std::to_string(contador));
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace visual_behavior