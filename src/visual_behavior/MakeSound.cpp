#include "visual_behavior/MakeSound.h"

#include "kobuki_msgs/Sound.h"

#include "ros/ros.h"

namespace visual_behavior
{

MakeSound::MakeSound(const std::string& name)
: BT::ActionNodeBase(name, {})
{
    pub_sound_ = n_.advertise<kobuki_msgs::Sound>("mobile_base/commands/sound", 1);
}
 
BT::NodeStatus
MakeSound::tick()
{
    kobuki_msgs::Sound sound;

    if (status() == BT::NodeStatus::IDLE)
    {
        ROS_INFO("Help");
    }
    
    sound = 2;
    pub_sound_.publish(sound);

    return BT::NodeStatus::SUCCESS;
}

}  // namespace visual_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behavior_trees::MakeSound>("MakeSound");
}