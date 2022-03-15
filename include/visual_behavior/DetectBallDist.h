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

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"



#include <string>

namespace visual_behavior
{

class DetectBall : public BT::ActionNodeBase
{
  public:
    explicit DetectBall(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick();
    void halt();

    static BT::PortsList providedPorts()
    {
        // This action has a single input port called "message"
        // Any port must have a name. The type is optional.
        return { BT::InputPort<std::string>("foward_direction") };
    }

  private:

    ros::NodeHandle n_;
    ros::Subscriber sub_darknet_;
};

}  // namespace visual_behavior

#endif  // VISUAL_BEHAVIOR_DETECTBALL_H