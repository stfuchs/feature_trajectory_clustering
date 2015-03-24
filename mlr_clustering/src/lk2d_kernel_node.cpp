/**
 * @file   lk2d_kernel_node.cpp
 * @author Steffen Fuchs <steffen@steffen-ubuntu>
 * @date   Wed Feb  4 22:13:49 2015
 * 
 * @brief  
 * 
 * 
 */


#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Bool.h>
#include <mlr_msgs/Point2dArray.h>
#include <mlr_msgs/KernelState.h>
#include <mlr_common/kernel.hpp>
#include <mlr_common/tracker_types.hpp>

struct LK2dKernelNode
{
  LK2dKernelNode()
  { 
    topic_traj_ = "lk2d/points";
    topic_out_ = "kernel";
    ROS_INFO("Default input Point2dArray topic is: %s", topic_traj_.c_str());
    sub = nh.subscribe(topic_traj_,1,&LK2dKernelNode::lkCallback,this);
    sub_reset = nh.subscribe("reset_all",1,&LK2dKernelNode::reset,this);

    ROS_INFO("Default output topic is: %s", topic_out_.c_str());
    pub = nh.advertise<mlr_msgs::KernelState>(topic_out_,1);
    nh.param<LK2d_Tracker::TimeT>("kernel/timespan", LK2d_Tracker::timespan, 10.);
    ROS_INFO("Selected time window for kernel: %f", LK2d_Tracker::timespan);
  }

  void reset(std_msgs::Bool const& msg = std_msgs::Bool())
  {
    if (msg.data)
    {
      ROS_INFO("RESET");
      kernel.reset();
    }
  }

  void lkCallback(mlr_msgs::Point2dArray const& msg)
  {
    if (msg.header.stamp < last_header.stamp)
    {
      reset();
    }

    last_header = msg.header;

    ros::Time start = ros::Time::now();
    for(size_t i=0; i<msg.ids.size(); ++i)
    {
      LK2d_Tracker::IdT id(msg.ids[i]);
      LK2d_Tracker::StateT p(msg.x[i]*msg.scale_x, msg.y[i]*msg.scale_y);
      LK2d_Tracker::TimeT t(msg.header.stamp.toSec());
      if (kernel.isNew(id)) kernel.newTrajectory(id,p,t);
      else kernel.updateTrajectory(id,p,t);
    }
    ROS_INFO("LK Update took %f", (ros::Time::now() - start).toSec());
    publishKernelState(msg.header);
  }

  void publishKernelState(const std_msgs::Header& header)
  {
    ros::Time start = ros::Time::now();
    mlr_msgs::KernelState ks;
    ks.header = header;
    kernel.collectGarbage<LK2d_Tracker>(header.stamp.toSec());
    kernel.computeKernelMatrixData(ks.data,ks.ids);
    std::cout<<"Kernel ids: "<<ks.ids.size()<<" data: "<<ks.data.size()<<std::endl;
    pub.publish(ks);
    ROS_INFO("LK Kernel matrix took %f", (ros::Time::now() - start).toSec());
  }


  std::string topic_traj_;
  std::string topic_out_;

  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Subscriber sub_reset;
  ros::Publisher pub;
  Kernel<LK2d_Tracker> kernel;
  std_msgs::Header last_header;
};

int main(int argc, char** argv)
{
  ros::init(argc,argv, "lk2d_kernel");
  LK2dKernelNode n;
  ros::Rate r(30);
  while (n.nh.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
