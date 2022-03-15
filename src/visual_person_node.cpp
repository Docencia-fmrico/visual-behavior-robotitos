#include <string>
#include <memory>

#include "ros/ros.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "ros/package.h"

#include "visual_behavior/MakeSound.h"
#include "visual_behavior/DetectPerson.h"
#include "visual_behavior/Foward.h"
#include "visual_behavior/Turn.h"
#include "visual_behavior/DetectPersonDist.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "visual_behavior");
  ros::NodeHandle n;

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerNodeType<visual_behavior::MakeSound>("MakeSound");
  factory.registerNodeType<visual_behavior::DetectPerson>("DetectPerson");
  factory.registerNodeType<visual_behavior::Foward>("Foward");
  factory.registerNodeType<visual_behavior::Turn>("Turn");
  factory.registerNodeType<visual_behavior::DetectPersonDist>("DetectPersonDist");

  auto blackboard = BT::Blackboard::create();
  
  std::string pkgpath = ros::package::getPath("visual_behavior");
  std::string xml_file = pkgpath + "/visual_behavior_xml/visual_person_mod.xml";

  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);
  auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 1666, 1667);

  ros::Rate loop_rate(10);

  int count = 0;

  while (ros::ok())
  {
    tree.rootNode()->executeTick();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
