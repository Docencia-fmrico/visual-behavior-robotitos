#ifndef VISUAL_BEHAVIOR_DETECTBALLDIST_H
#define VISUAL_BEHAVIOR_DETECTBALLDIST_H

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"

#include "visual_behavior/transforms.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"


namespace visual_behavior
{

class DetectBallDist : public BT::ActionNodeBase
{
  public:
    explicit DetectBallDist(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick();
    void halt();

  private:
    bool found_ball_;
    static constexpr double WALKING_TIME = 0.5;
    ros::Time detected_ts_;

    ros::NodeHandle n_;
    ros::Publisher pub_vel_;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(tf2_ros::Buffer buffer);

    geometry_msgs::TransformStamped bf2ball_msg;
    tf2::Stamped<tf2::Transform> bf2ball;
    std::string error;
};

}  // namespace visual_behavior

#endif  // VISUAL_BEHAVIOR_DETECTBALLDIST_H