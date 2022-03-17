#ifndef VISUAL_BEHAVIOR_DETECTBALLDIST_H
#define VISUAL_BEHAVIOR_DETECTBALLDIST_H

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "transforms.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"


namespace visual_behavior
{

class DetectBallDist : public BT::ActionNodeBase
{
  public:
    explicit DetectBallDist(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick();

  private:
    bool found_ball_;
    ros::NodeHandle n_;
    ros::Publisher pub_vel_;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener();

    geometry_msgs::TransformStamped bf2ball_msg;
    geometry_msgs::TransformStamped odom2bf_msg;
    geometry_msgs::TransformStamped odom2ball_msg;
    geometry_msgs::TransformStamped ball2odom_msg;

    tf2::Stamped<tf2::Transform> bf2ball;
    tf2::Stamped<tf2::Transform> odom2bf;
    tf2::Stamped<tf2::Transform> odom2ball;
    tf2::Stamped<tf2::Transform> ball2odom;

    tf2::Transform ball2bf;

    std::string error;
};

}  // namespace visual_behavior

#endif  // VISUAL_BEHAVIOR_DETECTBALLDIST_H