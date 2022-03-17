#include "visual_behavior/DetectBallDist.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
<<<<<<< HEAD
#include "geometry_msgs/Twist.h"

=======
>>>>>>> 3319df14f8080f0667c8ceac669914b1b0be15b6
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
<<<<<<< HEAD

#include "visual_behavior/transforms.h"
=======
#include "transforms.h"

#include "geometry_tf/transforms.h"
>>>>>>> 3319df14f8080f0667c8ceac669914b1b0be15b6

#include "ros/ros.h"
#include <string>

namespace visual_behavior
{

DetectBallDist::DetectBallDist(const std::string& name, const BT::NodeConfiguration & config): BT::ActionNodeBase(name, config)
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
  geometry_msgs::Twist cmd;
  if (buffer.canTransform("base_footprint", "object/0", ros::Time(0), ros::Duration(0.1), &error))
  {
      ROS_INFO("Loking for a ball dist");
  }

  tf2_ros::TransformListener listener(buffer);
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
  ros::spinOnce();
  loop_rate.sleep();

  return BT::NodeStatus::SUCCESS;
}

}  // namespace visual_behavior
