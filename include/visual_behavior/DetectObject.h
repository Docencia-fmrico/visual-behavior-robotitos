
// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BEHAVIOR_TREES_APPROACHOBJECT_H
#define BEHAVIOR_TREES_APPROACHOBJECT_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"

#include "kobuki_msgs/BumperEvent.h"
#include "geometry_msgs/Twist.h"

#include <string>

namespace behavior_trees
{

class ApproachObject : public BT::ActionNodeBase
{
  public:
    explicit ApproachObject(const std::string& name, const BT::NodeConfiguration& config);
    void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);

    void halt();

    BT::NodeStatus tick();


    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("object")};
    }

  private:
    static const int GOING_FORWARD = 0;
    static const int GOING_BACK = 1;
    static const int TURNING_LEFT = 2;
    static const int TURNING_RIGHT = 3;
    static const int LEFT_PRESSED = 0;
    static const int RIGHT_PRESSED = 1;
    static constexpr double TURNING_TIME = 3.0;
    static constexpr double BACKING_TIME = 3.0;

    int state_;

    bool pressed_;
    int pressed_state_;

    ros::NodeHandle n_;
  
    ros::Time press_ts_;
    ros::Time turn_ts_;
    ros::Publisher pub_vel_;
    ros::Subscriber sub_bumper_;
};

}  // namespace behavior_trees

#endif  // BEHAVIOR_TREES_APPROACHOBJECT_H