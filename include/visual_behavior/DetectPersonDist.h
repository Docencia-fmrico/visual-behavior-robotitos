#ifndef VISUAL_BEHAVIOR_DETECTPERSONDIST_H
#define VISUAL_BEHAVIOR_DETECTPERSONDIST_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "visual_behavior/PID.h"

#include "ros/ros.h"
#include <string>

namespace visual_behavior
{

class DetectPersonDist : public BT::ConditionNode
{
  public:
    explicit DetectPersonDist(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick();
    void callback_bbx(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes);
    void CounterCallBack(const darknet_ros_msgs::ObjectCount::ConstPtr& counter);
   
    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<std::string>("turn_velocity"), BT::OutputPort<std::string>("foward_velocity") };
    }
    
  private:
    ros::NodeHandle n_;

    message_filters::Subscriber<sensor_msgs::Image> image_depth_sub;
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;
    message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx;

    bool found_person_;
    ros::Subscriber sub_counter_;
    float dist;
    int px_min, px_max;
    int py, px;
};

}  // namespace visual_behavior

#endif  // VISUAL_BEHAVIOR_DETECTPERSONDIST_H