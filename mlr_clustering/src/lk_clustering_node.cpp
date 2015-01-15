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
#include <mlr_msgs/KernelState.h>
#include "mlr_common/kernel.hpp"
#include "mlr_common/tracker_types.hpp"
#include "mlr_clustering/object_prediction.hpp"

struct LK_ClusteringNode
{
  LK_ClusteringNode() : itopic("lk_kernel")
  { init(); }

  void init()
  {
    sub = nh.subscribe(itopic,1,&LK_ClusteringNode::lkCallback,this);
    // pub =
  }

  void lkCallback(mlr_msgs::KernelState const& kernel)
  {
    ros::Time start = ros::Time::now();
    size_t n = kernel.ids.size();
    Eigen::MatrixXf K = Eigen::MatrixXf::Ones(n,n);
    typename std::vector<float>::const_iterator it = kernel.data.begin();
    for(size_t i=0;i<n;++i) 
      for(size_t j=i+1;j<n; ++j)
        K(i,j) = K(j,i) = *it++;

    predictor.predict(K,kernel.ids);
    ros::Duration elapsed = ros::Time::now() - start;
    ROS_INFO("Prediction took %f", elapsed.toSec());
  }


  std::string itopic;


  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher pub;
  ObjectPrediction predictor;
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
