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
      for(size_t i=0; i<points_[0].size(); ++i)
        points_[0][i] = pc_msg->points[index(features_[1][i])].getVector3fMap();
    }

    if (!features_[0].empty())
    {
      std::vector<unsigned char> status;
      std::vector<float> err;
      if(prev_gray_.empty()) gray_image.copyTo(prev_gray_);
      // in: prevImg, in: nextImg, in: prevPts, in_out: nextPts, out: status, out: err,
      // winSize=Size(21,21), maxLevel=3, criteria, flags=0, minEigThreshold=1e-4
      cv::calcOpticalFlowPyrLK(prev_gray_, gray_image, features_[0], features_[1],
                               status, err, cv::Size(31,31), 3, termcrit, 0, 0.001);

      for(size_t i=0; i<points_[1].size(); ++i)
        points_[1][i] = pc_msg->points[index(features_[1][i])].getVector3fMap();

      publishFeatureImage(image_map, features_[1]);
      publishTrajectoryUpdates(pc_msg,status);
    }

    std::swap(features_[1], features_[0]);
    cv::swap(prev_gray_, gray_image); 
  }

  void publishTrajectoryUpdates(const PointCloudConstPtr& pc,
                                const std::vector<unsigned char>& status)
  {
    mlr_msgs::TrajectoryPointUpdateArray update_array;
    mlr_msgs::TrajectoryPointUpdate update;
    update.header.stamp = ros::Time::now();
    update.header.frame_id = pc->header.frame_id;
    bool need_init;

    for(size_t i=0; i<features_[1].size(); ++i)
    {
      if( status[i] == 0 ) { continue; }
      
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
    }
    if(!update_array.points.empty()) pub_.publish(update_array);
  }


  void publishFeatureImage(const cv::Mat& image_map, const std::vector<cv::Point2f>& features)
  {
    // create a copy from the image map and reshape
    cv::Mat image;
    image_map.copyTo(image);
    image.rows = 480;
    image.cols = 640;
    image.step[0] = 3*640;

    for (size_t i=0; i<features.size(); ++i)
      cv::circle( image, features[i], 6, cv::Scalar(0,255,0), -1, 8);

    sensor_msgs::ImagePtr img_msg = cv2msg(image,"bgr8");
    img_pub_.publish(img_msg);
  }


  inline sensor_msgs::ImagePtr cv2msg(const cv::Mat& mat, const std::string& encoding){
    return cv_bridge::CvImage(std_msgs::Header(),encoding,mat).toImageMsg();
  }

  inline size_t index(const cv::Point2d& feature, size_t width=640) {
    return round(feature.x) + round(feature.y)*width;
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
