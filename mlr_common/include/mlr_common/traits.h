/**
 * @file   traits.h
 * @author Steffen Fuchs <steffenfuchs@samson.informatik.uni-stuttgart.de>
 * @date   Wed Dec 10 12:32:08 2014
 * 
 * @brief  
 * 
 * 
 */

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/ros.h>

typedef unsigned int TrajectoryId;

struct DefaultTrajectoryTraits
{
  typedef float ValueT;
  typedef float StateT;
  typedef float TimeT;

  static const unsigned int n_min = 3;
  static const unsigned int n_max = 1000;
  static const TimeT timespan = 3.;
};


struct PointTrajectoryTraits : public DefaultTrajetoryTraits
{
  typedef Eigen::Matrix<ValueT,3,1> StateT;
  typedef ros::Time TimeT;
};


struct PoseTrajectoryTraits : public DefaultTrajetoryTraits
{
  struct StateT
  {
    Eigen::Matrix<ValueT,3,1> point;
    Eigen::Quaternion<ValueT> orientation;
  };
  typedef ros::Time TimeT;
};

