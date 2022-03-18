#include "visual_behavior/DetectBallDist.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/Twist.h"

#include "tf2/transform_datatypes.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"

#include "visual_behavior/transforms.h"

#include "ros/ros.h"
#include <string>

namespace visual_behavior
{

DetectBallDist::DetectBallDist(const std::string& name, const BT::NodeConfiguration & config): BT::ActionNodeBase(name, config), listener(buffer)
{
  found_ball_ = false;
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);
}

void
DetectBallDist::halt()
{
  ROS_INFO("DetectBallDisthalt");
}

BT::NodeStatus
DetectBallDist::tick()
{
  if (status() == BT::NodeStatus::IDLE)
  {
      ROS_INFO("Looking for a ball dist");
  }

  geometry_msgs::Twist cmd;
  
  if (buffer.canTransform("base_footprint", "object/0", ros::Time(0), ros::Duration(0.1), &error))
  {
    bf2ball_msg = buffer.lookupTransform("base_footprint", "object/0", ros::Time(0));

    tf2::fromMsg(bf2ball_msg, bf2ball);
    
    double dist = bf2ball.getOrigin().length();
    double angle = atan2(bf2ball.getOrigin().y(), bf2ball.getOrigin().x());

    //es la forma de obtener los valores de los ejes de rotacion
    //Se imprime los valores obtenidos antes de las coordenadas
    ROS_INFO("base_footprint -> ball [%lf, %lf] dist=%lf angle=%lf %lf ago", bf2ball.getOrigin().x(), bf2ball.getOrigin().y(), dist, angle, (ros::Time::now() - bf2ball.stamp_).toSec());

    cmd.linear.x = dist-1.0;
    cmd.angular.z = angle;
    std::cerr << "dist: " << dist << std::endl;

    detected_ts_ = ros::Time::now();
    while ((ros::Time::now() - detected_ts_).toSec() < WALKING_TIME) {
        pub_vel_.publish(cmd);
    }

    return BT::NodeStatus::SUCCESS;
  } else {
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    pub_vel_.publish(cmd);
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace visual_behavior
