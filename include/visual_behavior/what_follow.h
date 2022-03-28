#ifndef VISUAL_BEHAVIOR_WHAT_FOLLOW_H
#define VISUAL_BEHAVIOR_WHAT_FOLLOW_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float32.h>

#include <string>

namespace visual_behavior
{

class what_follow : public BT::ConditionNode
{
  public:
    explicit what_follow(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick();
    void DetectPersonCallBack(const darknet_ros_msgs::BoundingBoxes::ConstPtr& boxes);
    void CounterCallBack(const darknet_ros_msgs::ObjectCount::ConstPtr& counter);
    void DetectBallCallBack(const sensor_msgs::Image::ConstPtr& image);

  private:
    bool found_ball_;
    bool found_person_;
    int contador = 0;
    ros::NodeHandle n_;
    ros::Subscriber sub_hsv_;
    ros::Subscriber sub_counter_;
    ros::Subscriber sub_darknet_;
};

}  // namespace visual_behavior

#endif  // VISUAL_BEHAVIOR_WHAT_FOLLOW_H 