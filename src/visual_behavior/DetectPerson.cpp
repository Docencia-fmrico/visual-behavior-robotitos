#include "visual_behavior/Foward.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <darknet_ros_msgs/BoundingBoxes.h>

#include "ros/ros.h"

namespace visual_behavior
{

DetectPerson::DetectPerson(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config), counter_(0)
{
}

void
DetectPerson::halt()
{
  ROS_INFO("ApproachObject halt");
}

BT::NodeStatus
DetectPerson::tick()
{
  if (status() == BT::NodeStatus::IDLE)
  {
    ROS_INFO("First time in ApproachObject");
  }

  std::string object = getInput<std::string>("object").value();
  ROS_INFO("ApproachObject [%s] tick %d", object.c_str(), counter_);

  if (counter_++ < 5)
  {
    return BT::NodeStatus::RUNNING;
  }
  else
  {
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace visual_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behavior_trees::DetectPerson>("DetectPerson");
}