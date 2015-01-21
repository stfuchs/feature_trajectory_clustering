/**
 * @file   lucas_kanade_tracker.cpp
 * @author Steffen Fuchs <steffen@steffen-ubuntu>
 * @date   Mon Nov 10 15:36:15 2014
 * 
 * @brief  
 * 
 * 
 */


#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <mlr_msgs/TrajectoryPointUpdateArray.h>

struct LucasKanadeTrackerNode
{
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

  LucasKanadeTrackerNode() 
    : itopic_("camera/depth_registered/points") 
    , otopic_("lk_points")
    , it_(nh_)

  { init(); }

  void init()
  {
    ROS_INFO("Subscribing to %s", itopic_.c_str());
    sub_ = nh_.subscribe(itopic_,1,&LucasKanadeTrackerNode::callback,this);
    pub_ = nh_.advertise<mlr_msgs::TrajectoryPointUpdateArray>(otopic_,1);
    img_pub_ = it_.advertise("lk_image",1);
  }

  void callback(const PointCloudConstPtr& pc_msg)
  {
    std_msgs::Header header = pcl_conversions::fromPCL(pc_msg->header);
    if (header.stamp < last_header_.stamp)
    {
      ROS_INFO("Message timestamp older than previous: RESET");
      features_[0].clear();
      features_[1].clear();
      points_[0].clear();
      points_[1].clear();
    }
    last_header_ = header;

    // non-copy map to rgb data in point cloud:
    const char* pmsg = reinterpret_cast<const char*>(&(pc_msg->points[0]))+16;
    const cv::Mat image_map(480*640,1,CV_8UC3, const_cast<char*>(pmsg),32);
    // copy to gray image:
    cv::Mat gray_image;
    cv::cvtColor(image_map, gray_image, cv::COLOR_BGR2GRAY);
    gray_image.rows = 480; gray_image.cols = 640; gray_image.step[0] = 640;
    
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,20,0.03);

    if (features_[1].size() < 10)
    {
      // image_in, corners_out, max_corners, quality_level, min_distance,
      // mask, block_size, harris?, harris_param
      cv::goodFeaturesToTrack(gray_image, features_[1], 500, 0.01, 10, cv::Mat(), 3, false, 0.04);
      // subpixel accuracy fo corners:
      // image_in, corners_in_out, win_size, zero_zone, termination_criteria
      cv::cornerSubPix(gray_image, features_[1], cv::Size(10,10), cv::Size(-1,-1), termcrit);
      ROS_INFO("Initialized new features: %zu",features_[1].size());

      points_[0].resize(features_[1].size());
      points_[1].resize(features_[1].size());
      status_[0] = std::vector<unsigned char>(features_[1].size(),1);
      status_[1] = std::vector<unsigned char>(features_[1].size(),1);
      for(size_t i=0; i<points_[0].size(); ++i)
        points_[0][i] = subPixelInterpolation(features_[1][i], pc_msg);
      //points_[0][i] = pc_msg->points[index(features_[1][i])].getVector3fMap();
    }

    if (!features_[0].empty())
    {
      //std::vector<unsigned char> status;
      std::vector<float> err;
      if(prev_gray_.empty()) gray_image.copyTo(prev_gray_);
      // in: prevImg, in: nextImg, in: prevPts, in_out: nextPts, out: status, out: err,
      // winSize=Size(21,21), maxLevel=3, criteria, flags=0, minEigThreshold=1e-4
      cv::calcOpticalFlowPyrLK(prev_gray_, gray_image, features_[0], features_[1],
                               status_[1], err, cv::Size(31,31), 3, termcrit, 0, 0.001);

      for(size_t i=0; i<points_[1].size(); ++i)
        points_[1][i] = subPixelInterpolation(features_[1][i], pc_msg);
        //points_[1][i] = pc_msg->points[index(features_[1][i])].getVector3fMap();

      publishTrajectoryUpdates(image_map,pc_msg);
    }

    std::swap(features_[1], features_[0]);
    std::swap(status_[1], status_[0]);
    cv::swap(prev_gray_, gray_image); 
  }

  void publishTrajectoryUpdates(const cv::Mat& image_map,
                                const PointCloudConstPtr& pc)
  {
    cv::Mat image;
    image_map.copyTo(image);
    image.rows = 480;
    image.cols = 640;
    image.step[0] = 3*640;

    mlr_msgs::TrajectoryPointUpdateArray update_array;
    mlr_msgs::TrajectoryPointUpdate update;
    update.header = last_header_;
    bool need_init;

    for(size_t i=0; i<features_[1].size(); ++i)
    {
      if( status_[0][i] == 0 ) { status_[1][i] = 0; continue; }
      if( status_[1][i] == 0 ) { continue; }

      Eigen::Vector3f& p0 = points_[0][i];
      Eigen::Vector3f& p1 = points_[1][i];
      
      if( p0(2) != p0(2) ) { std::swap(p1,p0); continue; }
      if( p1(2) != p1(2) ) { continue; }
      if( (p0-p1).norm() > .1 ) { continue; }

      update.point.x = p1(0);
      update.point.y = p1(1);
      update.point.z = p1(2);
      update.id = i;
      update_array.points.push_back(update);
      std::swap(p1,p0);
      cv::circle( image, features_[1][i], 3, cv::Scalar(0,255,0), -1, 8);
    }
    if(!update_array.points.empty())
    {
      pub_.publish(update_array);
      sensor_msgs::ImagePtr img_msg
        = cv_bridge::CvImage(last_header_,"bgr8",image).toImageMsg();
      img_pub_.publish(img_msg);
    }
    else
    {
      ROS_INFO("no points tracked");
    }
  }

  inline size_t index(const cv::Point2d& feature, size_t width=640) {
    return round(feature.x) + round(feature.y)*width;
  }

  Eigen::Vector3f subPixelInterpolation(
    const cv::Point2d& feature, const PointCloudConstPtr& pc)
  {
    size_t idx = floor(feature.x) + floor(feature.y)*pc->width;
    float s = feature.x - floor(feature.x);
    float t = feature.y - floor(feature.y);
    Eigen::Vector3f p1, p2;
    Eigen::Vector3f p11 = (*pc)[idx].getVector3fMap();
    Eigen::Vector3f p12 = (*pc)[idx+1].getVector3fMap();
    Eigen::Vector3f p21 = (*pc)[idx+pc->width].getVector3fMap();
    Eigen::Vector3f p22 = (*pc)[idx+pc->width+1].getVector3fMap();

    if (p11(2) == p11(2) && p12(2) == p12(2))
      p1 = s*p12 + (1.-s)*p11;
    else if (p11(2) == p11(2))
      p1 = p11;
    else
      p1 = p12;

    if (p21(2) == p21(2) && p22(2) == p22(2))
      p2 = s*p22 + (1.-s)*p21;
    else if (p21(2) == p21(2))
      p2 = p21;
    else
      p2 = p22;

    if (p1(2) == p1(2) && p2(2) == p2(2))
      return t*p2 + (1.-t)*p1;
    else if (p1(2) == p1(2))
      return p1;
    else
      return p2;
  }

  std::string itopic_;
  std::string otopic_;

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  image_transport::ImageTransport it_;
  image_transport::Publisher img_pub_;

  cv::Mat prev_gray_;
  std::vector<cv::Point2f> features_[2];
  std::vector<Eigen::Vector3f> points_[2];
  std::vector<unsigned char> status_[2];

  std_msgs::Header last_header_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lucas_kanade_tracker");
  LucasKanadeTrackerNode n;
  ros::Rate r(30); // hz
  while (n.nh_.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
