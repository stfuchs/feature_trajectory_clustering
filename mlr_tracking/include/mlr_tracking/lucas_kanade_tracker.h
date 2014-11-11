/**
 * @file   lucas_kanade_tracker.h
 * @author Steffen Fuchs <steffen@steffen-ubuntu>
 * @date   Mon Nov 10 15:36:09 2014
 * 
 * @brief  
 * 
 * 
 */


#ifndef LUCAS_KANADE_TRACKER_H
#define LUCAS_KANADE_TRACKER_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <mlr_msgs/TrajectoryPointUpdateArray.h>

class LucasKanadeTrackerNode
{
public:
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

public:
  LucasKanadeTrackerNode()
    : nh_()
    , it_(nh_)
    , input_topic_("camera/depth_registered/points")
  { init(); }

  ~LucasKanadeTrackerNode() {}

  void init();

  void callback(const PointCloudConstPtr& pc_msg);

  void publishFeatureImage(const cv::Mat& image_map, const std::vector<cv::Point2f>& features);

  void publishTrajectoryUpdates(const PointCloudConstPtr& pc);


  inline sensor_msgs::ImagePtr cv2msg(const cv::Mat& mat, const std::string& encoding)
  {
    return cv_bridge::CvImage(std_msgs::Header(),encoding,mat).toImageMsg();
  }
  
  inline bool ok() { return nh_.ok(); }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  image_transport::ImageTransport it_;
  image_transport::Publisher img_pub_;

  std::string input_topic_;
  cv::Mat prev_gray_;
  std::vector<cv::Point2f> features_[2];
  std::vector<float> depth_jump_;
  
};

#endif
