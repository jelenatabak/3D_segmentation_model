#include "build_dataset.h"

BuildDataset::BuildDataset(ros::NodeHandle& nodeHandle) {
  nodeHandle_ = nodeHandle;

  message_filters::Subscriber<sensor_msgs::Image> orig_mask_sub(nodeHandle_, "/mask/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> predicted_mask_sub(nodeHandle_, "/segmentation_mask", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nodeHandle_, "/depth_registered/points", 1);
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud2> sync(orig_mask_sub, predicted_mask_sub, pc_sub, 10);
  sync.registerCallback(boost::bind(&BuildDataset::callback, this, _1, _2, _3));

  pc_pub_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 1);

  ROS_INFO("Constructor");
  ros::spin();
}

void BuildDataset::callback(const sensor_msgs::ImageConstPtr& mask, const sensor_msgs::ImageConstPtr& predicted_mask, const sensor_msgs::PointCloud2ConstPtr& pc) {
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
    pcl::PointCloud<PointT> pcLabeled;

    cv_bridge::CvImagePtr predictedPtr;
    cv_bridge::CvImagePtr origPtr;
    try{
      predictedPtr = cv_bridge::toCvCopy(predicted_mask);
      origPtr = cv_bridge::toCvCopy(mask);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat origMask = origPtr->image;
    cv::Mat predictedMask = predictedPtr->image;
    int r0 = 0,g0 = 0,b0 = 0;
    int r1 = 0,g1 = 0,b1 = 0;
    int counter = 0;
    for(int i = 0; i < predictedMask.cols; i++){
      for(int j = 0; j < predictedMask.rows; j++){
        cv::Vec3b rgbPixel0 = origMask.at<cv::Vec3b>(i,j);
        cv::Vec3b rgbPixel1 = predictedMask.at<cv::Vec3b>(i,j);
        r0 = rgbPixel1.val[0];
        g0 = rgbPixel1.val[1];
        b0 = rgbPixel1.val[2];
        if(b0 > 240){
          pcTransformed.at(j,i).r = 255;
          pcTransformed.at(j,i).g = 0;
          pcTransformed.at(j,i).b = 0;
          // ROS_INFO("%d  %d  %d", pcOriginal.at(i,j).r, pcOriginal.at(i,j).g, pcOriginal.at(i,j).b);
          pcLabeled.push_back(pcTransformed.at(j,i));  // umjesto ovog spremiti kao pandas, testirati s metodom za ucitavanje pd kao pc
          counter++;
        }
      }
    }
    //cv::namedWindow("image", CV_WINDOW_NORMAL);
    //cv::resizeWindow("image", 1024,1024);
    // cv::imshow("image", predictedMask);
    // cv::waitKey(0);
    // cv::destroyWindow("image");

    ROS_INFO("Total number of remaining points: %d", counter);

    sensor_msgs::PointCloud2 pcLabeledMsg;
    pcl::toROSMsg(pcLabeled, pcLabeledMsg);
    pcLabeledMsg.header.frame_id = "world";
    pc_pub_.publish(pcLabeledMsg);
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
  BuildDataset BuildDataset(nodeHandle);
}
