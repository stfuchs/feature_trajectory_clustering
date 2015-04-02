/**
 * @file   lk_kernel_node.cpp
 * @author Steffen Fuchs <steffen@steffen-ubuntu>
 * @date   Thu Jan 15 13:51:46 2015
 * 
 * @brief  responsible for keeping the kernel up-to-date
 * 
 * 
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <mlr_msgs/Point3dArray.h>
#include <mlr_msgs/KernelState.h>
#include <mlr_common/kernel.hpp>
#include <mlr_common/tracker_types.hpp>

struct LK3dKernelNode
{
  LK3dKernelNode()
  { 
    topic_traj_ = "lk3d/points";
    topic_out_ = "kernel";
    ROS_INFO("Default input Point3dArray topic is: %s", topic_traj_.c_str());
    sub = nh.subscribe(topic_traj_,1,&LK3dKernelNode::lkCallback,this);

    ROS_INFO("Default output topic is: %s", topic_out_.c_str());
    pub = nh.advertise<mlr_msgs::KernelState>(topic_out_,1);
    nh.param<double>("kernel/timespan", LK_Tracker::timespan, 5.);
    double vel09; // velocity to indicate motion
    nh.param<double>("kernel/vel09", vel09, 0.15);
    LK_Tracker::lambda = - log(0.1)/(vel09*vel09);

    double sig05; // standard deviation of distance to indicate difference
    nh.param<double>("kernel/sig05", sig05, 0.008);
    LK_Tracker::gamma = - log(0.5)/(sig05*sig05);
    
    ROS_INFO("Selected time window for kernel: %f", LK_Tracker::timespan);
    ROS_INFO("Selected lambda for kernel: %f", LK_Tracker::lambda);
    ROS_INFO("Selected gamma for kernel: %f", LK_Tracker::gamma);
  }

  void lkCallback(mlr_msgs::Point3dArray const& msg)
  {
    if (msg.header.stamp < last_header.stamp)
    {
      ROS_INFO("Message timestamp older than previous: RESET");
      kernel.reset();
    }
    last_header = msg.header;

    ros::Time start = ros::Time::now();
    for(size_t i=0; i<msg.ids.size(); ++i)
    {
      LK_Tracker::IdT id(msg.ids[i]);
      LK_Tracker::StateT p(msg.x[i], msg.y[i], msg.z[i]);
      LK_Tracker::TimeT t(msg.header.stamp.toSec());
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
    kernel.collectGarbage<LK_Tracker>(header.stamp.toSec());
    kernel.computeKernelMatrixData(ks.data,ks.ids);
    std::cout<<"Kernel ids: "<<ks.ids.size()<<" data: "<<ks.data.size()<<std::endl;
    pub.publish(ks);
    ROS_INFO("LK Kernel matrix took %f", (ros::Time::now() - start).toSec());
  }


  std::string topic_traj_;
  std::string topic_out_;

  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher pub;
  Kernel<LK_Tracker> kernel;
  std_msgs::Header last_header;
};

int main(int argc, char** argv)
{
  ros::init(argc,argv, "lk3d_kernel");
  LK3dKernelNode n;
  ros::Rate r(30);
  while (n.nh.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
