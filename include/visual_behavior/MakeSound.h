#ifndef VISUAL_BEHAVIOR_MAKESOUND_H
#define VISUAL_BEHAVIOR_MAKESOUND_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"

#include "kobuki_msgs/Sound.h"


namespace visual_behavior
{

class MakeSound : public BT::ActionNodeBase
{
  public:
    explicit MakeSound(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick();

  private:

    ros::NodeHandle n_;
    ros::Publisher pub_sound_;
};

}  // namespace visual_behavior

#endif  // VISUAL_BEHAVIOR_MAKESOUND_H