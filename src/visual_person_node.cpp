#include "visual_behavior/DetectObject.h"

#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "ros/package.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "visual_person");
  ros::NodeHandle n;

  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<behavior_trees::ADetectObject>("DetectObject");

  auto blackboard = BT::Blackboard::create();

  blackboard->set("object", "cup");

  std::string pkgpath = ros::package::getPath("visual_behavior");
  std::string xml_file = pkgpath + "/behavior_trees_xml/tree_1.xml";

  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  ros::Rate loop_rate(5);

  int count = 0;

  bool finish = false;
  while (ros::ok() && !finish)
  {
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
