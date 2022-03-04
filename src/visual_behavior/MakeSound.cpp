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
ApproachObject::tick()
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