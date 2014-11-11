/**
 * @file   lucas_kanade_tracker.cpp
 * @author Steffen Fuchs <steffen@steffen-ubuntu>
 * @date   Mon Nov 10 15:36:15 2014
 * 
 * @brief  
 * 
 * 
 */


#include "mlr_tracking/lucas_kanade_tracker.h"

void LucasKanadeTrackerNode::init()
{
  ROS_INFO("Subscribing to %s", input_topic_.c_str());
  sub_ = nh_.subscribe(input_topic_,1,&LucasKanadeTrackerNode::callback,this);
  pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("lk_points",1);
  img_pub_ = it_.advertise("/camera/image/mono",1);
}

void LucasKanadeTrackerNode::callback(const PointCloudConstPtr& pc_msg)
{
  // non-copy map to rgb data in point cloud:
  const void* pmsg = reinterpret_cast<const void*>(&(pc_msg->points[0]))+16;
  const cv::Mat color_image(480*640,1,CV_8UC3, const_cast<void*>(pmsg),32);
  // copy to gray image:
  cv::Mat gray_image;
  cv::cvtColor(color_image, gray_image, cv::COLOR_BGR2GRAY);
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

    /*
    size_t i,k;
    for (i=k=0; i<features_[1].size(); ++i)
    {
      if (!status[i]) continue;
      features_[1][k++] = features_[1][i];
    }
    features_[1].resize(k);
    ROS_INFO("Tracked features: %zu",k);
    */
    if(true)
    {
      cv::Mat image;
      color_image.copyTo(image);
      image.rows = 480;
      image.cols = 640;
      image.step[0] = 3*640;

      for (size_t i=0; i<features_[1].size(); ++i)
        cv::circle( image, features_[1][i], 3, cv::Scalar(0,255,0), -1, 8);

      sensor_msgs::ImagePtr img_msg = cv2msg(image,"bgr8");
      img_pub_.publish(img_msg);
    }
  }
  //typename pcl::PointCloud<pcl::PointXYZ>::Ptr pc_out(new pcl::PointCloud<pcl::PointXYZ>);
  //pc_out->header = pc_msg->header;
  //pub_.publish(pc_out);

  std::swap(features_[1], features_[0]);
  cv::swap(prev_gray_, gray_image); 
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "lucas_kanade_tracker");
  LucasKanadeTrackerNode n;
  ros::Rate r(30); // hz
  while (n.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
