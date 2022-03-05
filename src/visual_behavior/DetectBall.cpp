#include "visual_behavior/DetectBall.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <darknet_ros_msgs/BoundingBoxes.h>

#include "ros/ros.h"

namespace visual_behavior
{

DetectBall::DetectBall(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, {})
{
}

void
DetectBall::halt()
{
  ROS_INFO("DetectBall halt");
}

BT::NodeStatus
DetectBall::tick()
{
  if (status() == BT::NodeStatus::IDLE)
  {
    ROS_INFO("Loking for a person");
  }

  std::string object = getInput<std::string>("object").value();

  return BT::NodeStatus::SUCCESS;
}

}  // namespace visual_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<visual_behavior::DetectBall>("DetectBall");
}