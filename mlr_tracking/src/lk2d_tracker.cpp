/**
 * @file   lk2d_tracker.cpp
 * @author Steffen Fuchs <steffen@steffen-ubuntu>
 * @date   Wed Feb  4 10:53:43 2015
 * 
 * @brief  lucas kanade tracker for color images
 * 
 * 
 */

#include <unordered_map>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/video/tracking.hpp>

#include <mlr_msgs/Point2dArray.h>

struct LK2dTrackerNode
{
  LK2dTrackerNode() : it_(nh_)
  {
    itopic_ = "camera/rgb/image_color";
    otopic_ = "tracking/lk2d/points";
    ROS_INFO("Default input topic is: %s", itopic_.c_str());
    sub_img_ = it_.subscribe(itopic_, 1, &LK2dTrackerNode::callback, this);

    ROS_INFO("Default output topic is: %s", otopic_.c_str());
    //img_pub_ = it_.advertise("lk_image",1);
    pub_ = nh_.advertise<mlr_msgs::Point2dArray>(otopic_,1);
    id_count = 0;
  }

  void callback(const sensor_msgs::ImageConstPtr& img_msg)
  {
    img1 = cv_bridge::toCvCopy(img_msg,"mono8");
    if (!img0) img0 = img1;
    if (img1->header.stamp < img0->header.stamp)
    {
      ROS_INFO("Message timestamp older than previous: RESET");
      features.clear();
      id_count = 0;
    }

    std::vector<cv::Point2f> f0; // old features
    std::vector<cv::Point2f> f1; // new features
    std::vector<int> ids;
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,20,0.03);

    if (features.size() < 10)
    {
      cv::goodFeaturesToTrack(img1->image, f0, 300, 0.01, 10, cv::Mat(), 3, false, 0.04);
      cv::cornerSubPix(img1->image, f0, cv::Size(10,10), cv::Size(-1,-1), termcrit);
      ROS_INFO("Initialized new features: %zu",f0.size());
      for (size_t i=0; i<f0.size(); ++i) ids.push_back(id_count++);
    }

    typename std::unordered_map<int,cv::Point2f>::iterator it;
    for (it=features.begin(); it!=features.end(); ++it)
    {
      ids.push_back(it->first);
      f0.push_back(it->second);
    }

    std::vector<unsigned char> s;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(img0->image,img1->image,f0,f1,s,err,cv::Size(31,31),3,termcrit);

    mlr_msgs::Point2dArray msg;
    msg.header = img_msg->header;
    msg.scale_x = .5*float(img_msg->width);
    msg.scale_y = .5*float(img_msg->width);
    for (size_t i=0; i<f1.size(); ++i)
    {
      if (s[i] != 0)
      {
        features[ids[i]] = f1[i];
        msg.ids.push_back(ids[i]);
        msg.x.push_back(f1[i].x);
        msg.y.push_back(f1[i].y);
      }
      else
      {
        features.erase(ids[i]);
      }
    }
    if (!msg.ids.empty())
    {
      ROS_INFO("Published %zu features", msg.ids.size());
      pub_.publish(msg);
    }

    std::swap(img0,img1);
  }

  std::string itopic_;
  std::string otopic_;

  ros::NodeHandle nh_;
  ros::Publisher pub_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_img_;
  //image_transport::Publisher img_pub_;

  int id_count;
  cv_bridge::CvImagePtr img0; // old gray image
  cv_bridge::CvImagePtr img1; // new gray image
  std::unordered_map<int,cv::Point2f> features;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lk2d_tracker");
  LK2dTrackerNode n;
  ros::Rate r(30); // hz
  while (n.nh_.ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}

