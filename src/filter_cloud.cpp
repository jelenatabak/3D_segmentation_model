#include "filter_cloud.h"

FilterCloud::FilterCloud(ros::NodeHandle& nodeHandle) {
  nodeHandle_ = nodeHandle;

  message_filters::Subscriber<sensor_msgs::Image> image_sub(nodeHandle_, "/segmentation_mask", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nodeHandle_, "/depth_registered/points", 1);
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2> sync(image_sub, pc_sub, 10);
  sync.registerCallback(boost::bind(&FilterCloud::callback, this, _1, _2));

  pc_pub_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 1);

  ROS_INFO("Constructor");
  ros::spin();
}

void FilterCloud::callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::PointCloud2ConstPtr& pc) {
  ROS_INFO("In callback");

  // tf::StampedTransform transform;
  // try{
  //   listener_.lookupTransform("/world", "/camera", image->header.stamp, transform);
  //   //ROS_INFO("%d", transform.getOrigin().getX());
  // }
  // catch (tf::TransformException ex){
  //   ROS_ERROR("Error!");
  //   ros::Duration(1.0).sleep();
  // }

  pcl::PointCloud<PointT> pcOriginal;
  pcl::fromROSMsg(*pc, pcOriginal);
  pcl::PointCloud<PointT> pcTransformed;
  try{
    pcl_ros::transformPointCloud("/world", pcOriginal, pcTransformed, listener_);
    //pcl_ros::transformPointCloud(pcOriginal, pcTransformed, transform);
    pcl::PointCloud<PointT> pcFiltered;

    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(image);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat img = cv_ptr->image;
    int r = 0,g = 0,b = 0;
    int counter = 0;
    for(int i = 0; i < img.cols; i++){
      for(int j = 0; j < img.rows; j++){
        cv::Vec3b bgrPixel = img.at<cv::Vec3b>(i,j);
        r = bgrPixel.val[0];
        g = bgrPixel.val[1];
        b = bgrPixel.val[2];
        if(b > 240){
          // pcOriginal.at(j,i).r = 255;
          // pcOriginal.at(j,i).g = 0;
          // pcOriginal.at(j,i).b = 0;
          // ROS_INFO("%d  %d  %d", pcOriginal.at(i,j).r, pcOriginal.at(i,j).g, pcOriginal.at(i,j).b);
          pcFiltered.push_back(pcTransformed.at(j,i));
          counter++;
        }
      }
    }
    //cv::namedWindow("image", CV_WINDOW_NORMAL);
    //cv::resizeWindow("image", 1024,1024);
    // cv::imshow("image", img);
    // cv::waitKey(0);
    // cv::destroyWindow("image");

    ROS_INFO("Total number of remaining points: %d", counter);

    sensor_msgs::PointCloud2 pcFilteredMsg;
    pcl::toROSMsg(pcFiltered, pcFilteredMsg);
    pcFilteredMsg.header.frame_id = "world";
    pc_pub_.publish(pcFilteredMsg);
    ROS_INFO("Published!");
  }
  catch (...){
    ROS_ERROR("Error!");
    ros::Duration(1.0).sleep();
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "filter_cloud");
  ros::NodeHandle nodeHandle("");
  FilterCloud filterCloud(nodeHandle);
}
