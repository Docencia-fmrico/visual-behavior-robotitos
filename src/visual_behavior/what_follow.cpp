#include "visual_behavior/what_follow.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <darknet_ros_msgs/ObjectCount.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float32.h>

#include "ros/ros.h"
#include <string>

namespace visual_behavior
{

what_follow::what_follow(const std::string& name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config),found_person_(false)
{
    found_ball_ = false;
    sub_hsv_ = n_.subscribe("/hsv/image_filtered",1,&DetectBall::DetectBallCallBack,this);
    sub_counter_ = n_.subscribe("/darknet_ros/found_object", 1, &DetectPerson::CounterCallBack,this);
    sub_darknet_ = n_.subscribe("darknet_ros/bounding_boxes", 1, &DetectPerson::DetectPersonCallBack,this);
}

void
what_follow::DetectBallCallBack(const sensor_msgs::Image::ConstPtr& image) {
  for (const auto & pixel_value : image->data) {
     if (pixel_value != 0) {
        found_ball_ = true;
     } 
  }
}

void
what_follow::CounterCallBack(const darknet_ros_msgs::ObjectCount::ConstPtr& counter) {
  ROS_INFO(" callback counter");

  if (counter->count >= 1) {
    found_person_ = true;
  } else {
    found_person_ = false;
  }
}

void
what_follow::DetectPersonCallBack(const darknet_ros_msgs::BoundingBoxesConstPtr& boxes){
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
what_follow::tick()
{
  if (status() == BT::NodeStatus::IDLE)
  {
    ROS_INFO("loking for something");
  }

  if (found_ball_) {
    return BT::NodeStatus::SUCCESS;
  } else  if (found_person_){
    return BT::NodeStatus::FAILURE;
  } else {
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace visual_behavior