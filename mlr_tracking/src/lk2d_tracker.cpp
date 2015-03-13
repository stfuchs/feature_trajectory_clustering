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
      tracks.clear();
      id_count = 0;
    }

    std::vector<cv::Point2f> f0; // old features
    std::vector<cv::Point2f> f1f; // new features forward
    std::vector<cv::Point2f> f1b; // new features backward
    std::vector<int> ids;
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,10,0.03);

    typename std::unordered_map<int,cv::Point2f>::iterator it;
    for (it=tracks.begin(); it!=tracks.end(); ++it)
    {
      ids.push_back(it->first);
      f0.push_back(it->second);
    }

    if (!tracks.empty())
    {
      std::vector<unsigned char> s;
      std::vector<float> err;
      cv::calcOpticalFlowPyrLK(img0->image,img1->image,f0,f1f,s,err,cv::Size(31,31),3,termcrit);
      cv::calcOpticalFlowPyrLK(img1->image,img0->image,f1f,f1b,s,err,cv::Size(31,31),3,termcrit);

      mlr_msgs::Point2dArray msg;
      msg.header = img_msg->header;
      msg.scale_x = 1./float(img_msg->width);
      msg.scale_y = 1./float(img_msg->height);
      for (size_t i=0; i<f1f.size(); ++i)
      {
        cv::Point2f dw = f1f[i] - f1b[i];
        cv::Point2f wf = f1f[i] - f0[i];
        cv::Point2f wb = f1b[i] - f1f[i];
        if ( dw.x*dw.x+dw.y*dw.y < .5*(wf.x*wf.x + wf.y*wf.y + wb.x*wb.x + wb.y*wb.y)+.5 )
        {
          tracks[ids[i]] = f1f[i];
          msg.ids.push_back(ids[i]);
          msg.x.push_back(f1f[i].x);
          msg.y.push_back(f1f[i].y);
        }
        else
        {
          tracks.erase(ids[i]);
        }
      }
      if (!msg.ids.empty())
      {
        ROS_INFO("Published %zu tracks", msg.ids.size());
        pub_.publish(msg);
      }
    }

    if (tracks.size() < 50)
    {
      cv::goodFeaturesToTrack(img1->image, f0, 300, 0.01, 10, cv::Mat(), 3, false, 0.04);
      cv::cornerSubPix(img1->image, f0, cv::Size(10,10), cv::Size(-1,-1), termcrit);
      ROS_INFO("Initialized new tracks: %zu",f0.size());
      for (size_t i=0; i<f0.size(); ++i) ids.push_back(id_count++);
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
  std::unordered_map<int,cv::Point2f> tracks;
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

