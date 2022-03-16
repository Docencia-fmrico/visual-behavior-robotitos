#include "visual_behavior/DetectPersonDist.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include "visual_behavior/PID.h"

#include "ros/ros.h"
#include <string>

namespace visual_behavior
{

DetectPersonDist::DetectPersonDist(const std::string& name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config), sync_bbx(MySyncPolicy_bbx(10),image_depth_sub, bbx_sub)
{ found_person_ == false;

  image_depth_sub.subscribe(n_, "/camera/depth/image_raw", 1);
  bbx_sub.subscribe(n_, "/darknet_ros/bounding_boxes", 1);

  sub_counter_ = n_.subscribe("/darknet_ros/found_object", 1, &DetectPersonDist::CounterCallBack,this);
  sync_bbx.registerCallback(boost::bind(&DetectPersonDist::callback_bbx, this,  _1, _2));
}

void
DetectPersonDist::callback_bbx(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes){
  ROS_INFO(" callback detectperson dist");
  cv_bridge::CvImagePtr img_ptr_depth;

  try{
      img_ptr_depth = cv_bridge::toCvCopy(*image, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception:  %s", e.what());
      return;
  }
  px_max = image->width;
  px_min = 0;
  for (const auto & box : boxes->bounding_boxes) {
    if (box.Class == "person") {
      px = (box.xmax + box.xmin) / 2;
      int py = (box.ymax + box.ymin) / 2;
      found_person_ == false;
      dist = img_ptr_depth->image.at<float>(cv::Point(px, py)) * 0.001f;
    }
  }
}

void
DetectPersonDist::CounterCallBack(const darknet_ros_msgs::ObjectCount::ConstPtr& counter) {
  ROS_INFO(" callback counter");

  if (counter->count >= 1) {
    found_person_ = true;
  } else {
    found_person_ = false;
  }
}

BT::NodeStatus
DetectPersonDist::tick()
{
  if (status() == BT::NodeStatus::IDLE)
  {
    ROS_INFO("Loking for a person and return a distance");
  }
  PID pid_foward = PID(0.5, 6, -0.1, 0.2);
  PID pid_turn = PID(px_min, px_max, -0.3, 0.3);
  double foward_velocity = pid_foward.get_output(dist);
  double turn_velocity = pid_foward.get_output(px);

  std::cerr << "x:" << px << std::endl;
  std::cerr << "x_max:" << px_max << std::endl;
  if (found_person_ == true) {
    std::cerr << "dist:" << dist << std::endl;
    setOutput("foward_velocity", std::to_string(foward_velocity));
    setOutput("turn_velocity", std::to_string(turn_velocity));
    return BT::NodeStatus::SUCCESS;
  } else {
    setOutput("foward_velocity", "0.0" );
    setOutput("turn_velocity", "0.0" );
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace visual_behavior