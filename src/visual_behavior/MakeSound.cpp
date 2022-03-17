#include "visual_behavior/MakeSound.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "kobuki_msgs/Sound.h"

#include "ros/ros.h"
#include <string>

namespace visual_behavior
{

MakeSound::MakeSound(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
    pub_sound_ = n_.advertise<kobuki_msgs::Sound>("mobile_base/commands/sound", 1);
}

void
MakeSound::halt()
{
  ROS_INFO("MakeSound halt");
}

BT::NodeStatus
MakeSound::tick()
{
    kobuki_msgs::Sound sound;

    if (status() == BT::NodeStatus::IDLE)
    {
        ROS_INFO("Help");
    }
    
    BT::Optional<std::string>  counter = getInput<std::string>("counter");

    if (!counter)
    {
        throw BT::RuntimeError("missing required input [message]: ", counter.error() );
    } 

    sound.value = sound.ERROR;
    if (std::stoi(counter.value()) < 10) {
        return BT::NodeStatus::SUCCESS;
    } else {
        pub_sound_.publish(sound);
        ros::Duration(3.0).sleep();
        return BT::NodeStatus::RUNNING;
    }
}

}  // namespace visual_behavior
