/**
 * @file   experimental_depth_lucas_kanade_tracker.cpp
 * @author Steffen Fuchs <steffenfuchs@samson.informatik.uni-stuttgart.de>
 * @date   Wed Nov 12 14:27:27 2014
 * 
 * @brief  experimental lucas kanade tracker that uses depth images for tracking
 * 
 * 
 */


#include "mlr_tracking/experimental_depth_lucas_kanade_tracker.h"

void DepthLucasKanadeTrackerNode::init()
{
  ROS_INFO("Subscribing to %s", input_topic_.c_str());
  sub_ = nh_.subscribe(input_topic_,1,&DepthLucasKanadeTrackerNode::callback,this);
  pub_ = nh_.advertise<mlr_msgs::TrajectoryPointUpdateArray>("dlk_points",1);
  img_pub_ = it_.advertise("dlk_image",1);
  klt_.setNumberOfKeypoints(2000);
}

void DepthLucasKanadeTrackerNode::callback(const PointCloudConstPtr& pc_msg)
{
  // this seems necessary to force certain pcl algorithms to use depth data only:
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_depth(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*pc_msg,*pc_depth);

  klt_.setInputCloud(pc_msg);

  //if (keypoints_prev_->points.empty()) keypointsAgast(pc_msg,keypoints_prev_);
  if (!keyindices_prev_)
  {
    keypointsHarris(pc_msg,keyindices_prev_);
    klt_.setPointsToTrack(keyindices_prev_);
  }

  //pcl::tracking::PyramidalKLTTracker<pcl::PointXYZ> klt;

  klt_.compute();
  pcl::copyPointCloud(*klt_.getTrackedPoints(), *keypoints_new_);
  ROS_INFO("KLT tracked %zu keypoints",keypoints_new_->size());
  publishKeypointImage(pc_msg, keypoints_new_, klt_.getPointsToTrackStatus());

}

void DepthLucasKanadeTrackerNode::keypointsHarris(
  const pcl::PointCloud<Point>::ConstPtr& pc_in,
  pcl::PointIndicesConstPtr& keyindices)
{
  pcl::HarrisKeypoint2D<Point, pcl::PointXYZI> harris;
  harris.setInputCloud(pc_in);
  harris.setNumberOfThreads(6);
  harris.setNonMaxSupression(true);
  harris.setRadiusSearch(0.01);
  harris.setMethod(pcl::HarrisKeypoint2D<Point,pcl::PointXYZI>::TOMASI);
  harris.setThreshold(0.05);
  harris.setWindowWidth(5);
  harris.setWindowHeight(5);
  pcl::PointCloud<pcl::PointXYZI>::Ptr response(new pcl::PointCloud<pcl::PointXYZI>);
  harris.compute(*response);
  keyindices = harris.getKeypointsIndices();
  ROS_INFO("extracted %zu keypoints using Harris", keyindices->indices.size());
}

void DepthLucasKanadeTrackerNode::keypointsAgast(
  const pcl::PointCloud<Point>::ConstPtr& pc_in,
  pcl::PointCloud<pcl::PointUV>::Ptr& keypoints)
{
  pcl::AgastKeypoint2D<Point> agast;
  agast.setThreshold(80);
  agast.setInputCloud(pc_in);
  agast.compute(*keypoints);
  ROS_INFO("extracted %zu keypoints using AGAST", keypoints->points.size());
}

void DepthLucasKanadeTrackerNode::publishKeypointImage(
  const PointCloudConstPtr& pc,
  const pcl::PointCloud<pcl::PointUV>::Ptr& keypoints,
  const pcl::PointIndicesConstPtr& status)
{
  // non-copy map to depth data in point cloud:
  size_t w = pc->width;  // 640
  size_t h = pc->height; // 480
  cv::Mat depth_image(h,w,CV_8UC3);
  for(size_t i=0; i<h; ++i)
  {
    for(size_t j=0; j<w; ++j)
    {
      int z;
      if(pc->points[i*w+j].z != pc->points[i*w+j].z)
        z = 255;
      else
        z = std::min( pc->points[i*w+j].z/5.*255., 255. );

      depth_image.at<cv::Vec3b>(i,j) = cv::Vec3b(z,z,z);
    }
  }

  for(size_t i=0; i<keypoints->size(); ++i)
  {
    cv::Point2i p(round(keypoints->points[i].u), round(keypoints->points[i].v));
    if (status->indices[i] < 0)
      cv::circle(depth_image, p, 6, cv::Scalar(0,0,255), 1, 8);
    else
      cv::circle(depth_image, p, 6, cv::Scalar(0,255,0), 1, 8);
  }

  sensor_msgs::ImagePtr img_msg = cv2msg(depth_image, "bgr8");
  img_pub_.publish(img_msg);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_lucas_kanade_tracker");
  DepthLucasKanadeTrackerNode n;
  ros::Rate r(30); // hz
  while (n.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
