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
  pub_ = nh_.advertise<mlr_msgs::TrajectoryPointUpdateArray>("lk_points",1);
  img_pub_ = it_.advertise("lk_image",1);
}

void LucasKanadeTrackerNode::callback(const PointCloudConstPtr& pc_msg)
{
  // non-copy map to rgb data in point cloud:
  const void* pmsg = reinterpret_cast<const void*>(&(pc_msg->points[0]))+16;
  const cv::Mat image_map(480*640,1,CV_8UC3, const_cast<void*>(pmsg),32);
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
      publishFeatureImage(image_map, features_[1]);
    }
    publishTrajectoryUpdates(pc_msg);
  }
  //typename pcl::PointCloud<pcl::PointXYZ>::Ptr pc_out(new pcl::PointCloud<pcl::PointXYZ>);
  //pc_out->header = pc_msg->header;
  //pub_.publish(pc_out);

  std::swap(features_[1], features_[0]);
  cv::swap(prev_gray_, gray_image); 
}

void LucasKanadeTrackerNode::publishFeatureImage(const cv::Mat& image_map,
                                                 const std::vector<cv::Point2f>& features)
{
  // create a copy from the image map and reshape
  cv::Mat image;
  image_map.copyTo(image);
  image.rows = 480;
  image.cols = 640;
  image.step[0] = 3*640;

  for (size_t i=0; i<features.size(); ++i)
    cv::circle( image, features[i], 3, cv::Scalar(0,255,0), -1, 8);

  sensor_msgs::ImagePtr img_msg = cv2msg(image,"bgr8");
  img_pub_.publish(img_msg);
}

void LucasKanadeTrackerNode::publishTrajectoryUpdates(const PointCloudConstPtr& pc)
{
  mlr_msgs::TrajectoryPointUpdateArray update_array;
  mlr_msgs::TrajectoryPointUpdate update;
  update.header.stamp = ros::Time::now();
  update.header.frame_id = pc->header.frame_id;
  bool need_init;
  if(need_init = depth_jump_.empty())
  {
    depth_jump_.resize(features_[1].size());
  }
  for(size_t i=0; i<features_[1].size(); ++i)
  {
    if(cv::norm(features_[0][i] - features_[1][i]) > 1)
    {
      size_t idx = round(features_[1][i].x) + round(features_[1][i].y)*pc->width;
      update.point.z = pc->points[idx].z;
      if(update.point.z == update.point.z) // nan check
      {
        if(need_init || (depth_jump_[i] - update.point.z) < .001)
        {
          depth_jump_[i] = update.point.z;
          update.point.x = pc->points[idx].x;
          update.point.y = pc->points[idx].y;
          update.id = i;
          update_array.points.push_back(update);
        }
      }
    }
  }
  if(!update_array.points.empty())
    pub_.publish(update_array);
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
