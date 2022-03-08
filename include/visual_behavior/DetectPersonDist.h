#ifndef VISUAL_BEHAVIOR_DETECTPERSONDIST_H
#define VISUAL_BEHAVIOR_DETECTPERSONDIST_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include "ros/ros.h"
#include <string>

namespace visual_behavior
{

class DetectPersonDist : public BT::ActionNodeBase
{
  public:
    explicit DetectPersonDist(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick();
    void halt();

    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<std::string>("foward_direction"), BT::OutputPort<std::string>("foward_velocity") };
    }

  private:
    bool found_person_;
    float dist;
    
    ros::NodeHandle n_;
    ros::Subscriber sub_darknet_;
};

}  // namespace visual_behavior

#endif  // VISUAL_BEHAVIOR_DETECTPERSONDIST_H