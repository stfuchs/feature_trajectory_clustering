/**
 * @file   experimental_depth_lucas_kanade_tracker.h
 * @author Steffen Fuchs <steffenfuchs@samson.informatik.uni-stuttgart.de>
 * @date   Wed Nov 12 14:11:04 2014
 * 
 * @brief  experimental lucas kanade tracker that uses depth images for tracking
 * 
 * 
 */


#ifndef DEPTH_LUCAS_KANADE_TRACKER_H
#define DEPTH_LUCAS_KANADE_TRACKER_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <pcl/point_types.h>
#include <pcl/keypoints/agast_2d.h>
#include <pcl/keypoints/harris_2d.h>
#include <pcl/tracking/pyramidal_klt.h>

#include <opencv2/imgproc/imgproc.hpp>

#include "mlr_msgs/TrajectoryPointUpdateArray.h"

class DepthLucasKanadeTrackerNode
{
public:
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

public:
  DepthLucasKanadeTrackerNode()
    : nh_()
    , it_(nh_)
    , input_topic_("camera/depth_registered/points")
    , klt_(5,7,7)
    , keypoints_prev_(new pcl::PointCloud<pcl::PointUV>)
    , keypoints_new_(new pcl::PointCloud<pcl::PointUV>)
  { init(); }

  ~DepthLucasKanadeTrackerNode() { }

  void init();

  void callback(const PointCloudConstPtr& pc_msg);

  void keypointsHarris(const pcl::PointCloud<Point>::ConstPtr& pc_in,
                         pcl::PointIndicesConstPtr& keyindices);


  void keypointsAgast(const pcl::PointCloud<Point>::ConstPtr& pc_in,
                      pcl::PointCloud<pcl::PointUV>::Ptr& keypoints);

  void publishKeypointImage(const PointCloudConstPtr& pc,
                            const pcl::PointCloud<pcl::PointUV>::Ptr& keypoints,
                            const pcl::PointIndicesConstPtr& status);



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
  pcl::tracking::PyramidalKLTTracker<Point> klt_;
  pcl::PointCloud<pcl::PointUV>::Ptr keypoints_prev_;
  pcl::PointCloud<pcl::PointUV>::Ptr keypoints_new_;
  pcl::PointIndicesConstPtr keyindices_prev_;
  pcl::PointIndicesConstPtr keyindices_new_;
};


#endif
