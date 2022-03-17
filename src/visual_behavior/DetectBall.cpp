#include "visual_behavior/DetectBall.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
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

DetectBall::DetectBall(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
  found_ball_ = false;
  contador = 0;
  sub_hsv_ = n_.subscribe("/hsv/image_filtered",1,&DetectBall::DetectBallCallBack,this);
}

void
DetectBall::DetectBallCallBack(const sensor_msgs::Image::ConstPtr& image) {
  for (const auto & pixel_value : image->data) {
     if (pixel_value != 0) {
        found_ball_ = true;
     } 
  }
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
    ROS_INFO("Looking for a ball");
  }
  contador++;
  if (found_ball_) {
    contador = 0;
    setOutput("counter", "0");
    return BT::NodeStatus::SUCCESS;
  } else {
    setOutput("turn_velocity", "0.1" );
    setOutput("counter", "0");
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace visual_behavior