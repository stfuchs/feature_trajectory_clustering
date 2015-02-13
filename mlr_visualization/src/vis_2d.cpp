/**
 * @file   vis_2d.cpp
 * @author Steffen Fuchs <steffen@steffen-ubuntu>
 * @date   Wed Feb  4 13:36:46 2015
 * 
 * @brief  Visualizer for 2D Point Trajectories
 * 
 * 
 */

#include <queue>
#include <unordered_map>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Bool.h>
#include <mlr_msgs/Point2dArray.h>
#include <mlr_msgs/ObjectIds.h>
#include <mlr_visualization/utils.h>

#include <opencv2/imgproc/imgproc.hpp>

struct Vis2dNode
{
  Vis2dNode() : nh_(), it_(nh_)
  {
    topic_img_  = "camera/rgb/image_color";
    topic_traj_ = "tracking/lk2d/points";
    topic_obj_  = "tracking/objects";
    topic_out_  = "tracking/lk2d/image";
    ROS_INFO("Default input image topic is: %s", topic_img_.c_str());
    ROS_INFO("Default input Point2dArray topic is: %s", topic_traj_.c_str());
    ROS_INFO("Default input ObjectIds topic is: %s", topic_obj_.c_str());
    sub_img_ = it_.subscribe(topic_img_, 10, &Vis2dNode::image_cb, this);
    sub_traj_ = nh_.subscribe(topic_traj_, 1, &Vis2dNode::trajectory_cb, this);
    sub_objects_ = nh_.subscribe(topic_obj_, 1, &Vis2dNode::object_cb, this);
    sub_reset_ = nh_.subscribe("tracking/reset_all", 1, &Vis2dNode::reset, this);

    ROS_INFO("Default output image topic is: %s", topic_out_.c_str());
    pub_img_ = it_.advertise(topic_out_,1);
  }

  void reset(const std_msgs::Bool& msg = std_msgs::Bool())
  {
    if (msg.data)
    {
      std::queue<cv_bridge::CvImageConstPtr> empty;
      std::swap(empty,img_queue);
      colors.clear();
    }
  }

  void image_cb(const sensor_msgs::ImageConstPtr& img_msg)
  {
    img_queue.push(cv_bridge::toCvShare(img_msg, "bgr8"));
    ros::Duration d = img_msg->header.stamp - img_queue.front()->header.stamp;
    while(d.toSec() > 5. && !img_queue.empty())
    {
      img_queue.pop();
      d = img_msg->header.stamp - img_queue.front()->header.stamp;
    }
  }

  void trajectory_cb(const mlr_msgs::Point2dArray& msg)
  {
    cv_bridge::CvImagePtr img(new cv_bridge::CvImage);
    while(!img_queue.empty())
    {
      if (img_queue.front()->header.stamp >= msg.header.stamp)
      {
        img->image = img_queue.front()->image;
        img->header = img_queue.front()->header;
        img->encoding = img_queue.front()->encoding;
        break;
      }
      else
      {
        img_queue.pop();
      }
    }
    if(img_queue.empty())
    {
      ROS_WARN("Image Queue empty! No image received yet.");
      return;
    }

    for (size_t i=0; i<msg.ids.size(); ++i)
    {
      Visualization::Utils::ColorRGB c = colors[msg.ids[i]];
      cv::Point2f p = cv::Point2f(msg.x[i],msg.y[i]);
      cv::circle(img->image,p,5,cv::Scalar(0,0,0),-1,CV_AA);
      cv::circle(img->image,p,3,cv::Scalar(c.b,c.g,c.r),-1,CV_AA);
    }
    pub_img_.publish(img->toImageMsg());
    ROS_INFO("Visualized %zu trajectory updates", msg.ids.size());
  }

  void object_cb(const mlr_msgs::ObjectIds& msg)
  {
    for(int i=0; i<int(msg.offsets.size())-1; ++i)
    {
      Visualization::Utils::ColorRGB c;
      int l = msg.labels[i];
      if (l<Visualization::Utils::N)
        c = Visualization::Utils::palette[l];

      for(int j=msg.offsets[i]; j<msg.offsets[i+1]; ++j)
      {
        int id = msg.ids[j] >> 32;
        colors[id] = c;
      }
    }
  }

  std::string topic_img_;
  std::string topic_traj_;
  std::string topic_obj_;
  std::string topic_out_;
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_img_;
  image_transport::Publisher pub_img_;
  ros::Subscriber sub_traj_;
  ros::Subscriber sub_objects_;
  ros::Subscriber sub_reset_;

  std::queue<cv_bridge::CvImageConstPtr> img_queue;
  std::unordered_map<int,Visualization::Utils::ColorRGB> colors;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "vis_2d");
  Vis2dNode n;
  ros::Rate r(30); // hz
  while (n.nh_.ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}
