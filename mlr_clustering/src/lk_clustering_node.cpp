/**
 * @file   lk_clustering_node.cpp
 * @author Steffen Fuchs <steffenfuchs@samson.informatik.uni-stuttgart.de>
 * @date   Tue Jan 13 11:54:29 2015
 * 
 * @brief  
 * 
 * 
 */

#include <ros/ros.h>

#include <mlr_msgs/TrajectoryPointUpdateArray.h>
#include "mlr_common/kernel.hpp"
#include "mlr_common/tracker_types.hpp"
#include "mlr_clustering/object_prediction.hpp"

struct LK_ClusteringNode
{
  //typedef LK_Tracker::IdT LK_id;
  //typedef LK_
  LK_ClusteringNode() : itopic("lk_points")
  { init(); }

  void init()
  {
    sub = nh.subscribe(itopic,1,&LK_ClusteringNode::lkCallback,this);
    // pub =
  }

  void lkCallback(mlr_msgs::TrajectoryPointUpdateArray const& updated)
  {
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
    MultiType<std::vector,type_array<LK_Tracker::IdT> > ids;
    Eigen::MatrixXf K;
    kernel.computeKernelMatrix(K,ids);
    predictor.predict(K,ids);
    ros::Duration elapsed = ros::Time::now() - start;
    ROS_INFO("LK Update took %f", elapsed.toSec());
  }


  std::string itopic;


  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher pub;
  Kernel<LK_Tracker> kernel;
  ObjectPrediction<LK_Tracker> predictor;
};

int main(int argc, char** argv)
{
  ros::init(argc,argv, "lk_clustering");
  LK_ClusteringNode n;
  ros::Rate r(30);
  while (n.nh.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
