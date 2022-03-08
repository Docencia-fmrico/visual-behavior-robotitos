#include "visual_behavior/DetectPersonDist.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include "ros/ros.h"
#include <string>

namespace visual_behavior
{

DetectPersonDist::DetectPersonDist(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
  found_person_ = false;
}

void DetectPersonDistCallBack(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes) {
  cv_bridge::CvImagePtr img_ptr_depth;

  try{
      img_ptr_depth = cv_bridge::toCvCopy(*image, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception:  %s", e.what());
      return;
  }
  
  for (const auto & box : boxes->bounding_boxes) {
    int px = (box.xmax + box.xmin) / 2;
    int py = (box.ymax + box.ymin) / 2;

    float dist = img_ptr_depth->image.at<float>(cv::Point(px, py)) * 0.001f;
    std::cerr << box.Class << " at (" << dist << std::endl;
  }
}

void
DetectPersonDist::halt()
{
  ROS_INFO("DetectPersonDist halt");
}

BT::NodeStatus
DetectPersonDist::tick()
{
  if (status() == BT::NodeStatus::IDLE)
  {
    ROS_INFO("Loking for a person and return a distance");
  }

  message_filters::Subscriber<sensor_msgs::Image> image_depth_sub(n_, "/camera/depth/image_raw", 1);
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub(n_, "/darknet_ros/bounding_boxes", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;
  message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx(MySyncPolicy_bbx(10), image_depth_sub, bbx_sub);

  sync_bbx.registerCallback(boost::bind(&DetectPersonDistCallBack, _1, _2));

  if (found_person_ == true) {
    if (dist >= 0.6 && dist <= 1.0) {
      setOutput("foward_direction", "go" );
      setOutput("foward_velocity", "0.2" );
    } else if (dist > 1.0) {
      setOutput("foward_direction", "go" );
      setOutput("foward_velocity", "0.4" );
    } else if (dist <= 0.5) {
      setOutput("foward_direction", "back" );
      setOutput("foward_velocity", "0.1" );
    } else {
      setOutput("foward_direction", "back" );
      setOutput("foward_velocity", "0.0" );
    }
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace visual_behavior