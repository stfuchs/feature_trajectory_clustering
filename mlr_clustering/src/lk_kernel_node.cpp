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

#include <mlr_msgs/TrajectoryPointUpdateArray.h>
#include <mlr_msgs/KernelState.h>
#include <mlr_common/kernel.hpp>
#include <mlr_common/tracker_types.hpp>

struct LK_KernelNode
{
  LK_KernelNode() : itopic("lk_points"), otopic("lk_kernel")
  { init(); }

  void init()
  {
    sub = nh.subscribe(itopic,1,&LK_KernelNode::lkCallback,this);
    pub = nh.advertise<mlr_msgs::KernelState>(otopic,1);
  }

  void lkCallback(mlr_msgs::TrajectoryPointUpdateArray const& updated)
  {
    if (updated.points[0].header.stamp < last_header.stamp)
    {
      ROS_INFO("Message timestamp older than previous: RESET");
      kernel.reset();
    }


    last_header = updated.points[0].header;

    ros::Time start = ros::Time::now();
    typename std::vector<mlr_msgs::TrajectoryPointUpdate>::const_iterator it;
    for(it = updated.points.begin(); it != updated.points.end(); ++it)
    {
      LK_Tracker::IdT id(it->id);
      LK_Tracker::StateT p(it->point.x, it->point.y, it->point.z);
      LK_Tracker::TimeT t(it->header.stamp.toSec());
      if (kernel.isNew(id)) kernel.newTrajectory(id,p,t);
      else kernel.updateTrajectory(id,p,t);
    }
    ROS_INFO("LK Update took %f", (ros::Time::now() - start).toSec());
    publishKernelState(updated.points[0].header);
  }

  void publishKernelState(const std_msgs::Header& header)
  {
    ros::Time start = ros::Time::now();
    mlr_msgs::KernelState ks;
    ks.header = header;
    kernel.computeKernelMatrixData(ks.data,ks.ids);
    std::cout<<"Kernel ids: "<<ks.ids.size()<<" data: "<<ks.data.size()<<std::endl;
    pub.publish(ks);
    ROS_INFO("LK Kernel matrix took %f", (ros::Time::now() - start).toSec());
  }


  std::string itopic;
  std::string otopic;

  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher pub;
  Kernel<LK_Tracker> kernel;
  std_msgs::Header last_header;
};

int main(int argc, char** argv)
{
  ros::init(argc,argv, "lk_kernel");
  LK_KernelNode n;
  ros::Rate r(30);
  while (n.nh.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
