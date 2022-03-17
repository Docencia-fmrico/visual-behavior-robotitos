#ifndef VISUAL_BEHAVIOR_DETECTBALL_H
#define VISUAL_BEHAVIOR_DETECTBALL_H

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

class DetectBall : public BT::ActionNodeBase
{
  public:
    explicit DetectBall(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick();
    void halt();
    void DetectBallCallBack(const sensor_msgs::Image::ConstPtr& image);

    static BT::PortsList providedPorts()
    {
        return{ BT::OutputPort<std::string>("turn_velocity"), BT::OutputPort<std::string>("counter")};
    }

  private:
    bool found_ball_;
    int contador;
    ros::NodeHandle n_;
    ros::Subscriber sub_hsv_;
};

}  // namespace visual_behavior

#endif  // VISUAL_BEHAVIOR_DETECTBALL_H 