#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

#include "pcl_ros/transforms.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <tf/transform_listener.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std;

typedef pcl::PointXYZRGB PointT;

class FilterCloud{
public:
  FilterCloud(ros::NodeHandle& nodeHandle);
  void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::PointCloud2ConstPtr& pc);

private:
  ros::NodeHandle nodeHandle_;
  ros::Publisher pc_pub_;
  tf::TransformListener listener_;
};
