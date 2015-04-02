/**
 * @file   trajectory.h
 * @author Steffen Fuchs <st.fuchs.tr@gmail.com>
 * @date   Tue Dec  9 16:31:09 2014
 * 
 * @brief  
 * 
 * 
 */

#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include <deque>
#include <unordered_map>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "mlr_common/base_id.hpp"

template<typename Traits, typename Tracker>
struct Trajectory
{
  typedef Traits type;
  typedef Tracker tracker_type;
  typedef typename Traits::StateT StateT;
  typedef typename Traits::ValueT ValueT;
  typedef typename Traits::TimeT TimeT;
  typedef BaseId<Tracker> IdT;

  Trajectory(IdT const& id_) : id(id_) {}

  IdT const id;
  std::deque<TimeT> t;
  std::deque<StateT> x;
  std::deque<ValueT> w;
};

template<typename T>
inline typename T::ValueT computeWeight(
  typename T::StateT const& x1, typename T::StateT const& x2, 
  typename T::TimeT const& t1, typename T::TimeT const& t2)
{
  //return 1.;
  typename T::StateT v = (x2-x1)/(t2-t1);
  return 1.-exp(-T::lambda*v.dot(v)-T::eps);
}


/***************************************************************************************
 *  Predefined Traits
 **************************************************************************************/
struct DefaultTraits
{
  typedef double ValueT;
  typedef double StateT;
  typedef double TimeT;
};

struct PointTraits : DefaultTraits
{
  typedef Eigen::Matrix<double,3,1> StateT;
};


struct Point2dTraits : DefaultTraits
{
  typedef Eigen::Matrix<double,2,1> StateT;
};


struct PoseTraits : DefaultTraits
{
  typedef Eigen::Matrix<double,3,1> PointT;
  typedef Eigen::Quaternion<double> OrientationT;
  
  struct StateT
  {
    PointT point;
    OrientationT orientation;
  };
};

/*
template<>
inline PoseTraits::ValueT computeWeight<PoseTraits>(
  PoseTraits::StateT const& x1, PoseTraits::StateT const& x2,
  PoseTraits::TimeT const& t1, PoseTraits::TimeT const& t2)
{
  PoseTraits::PointT v = 100.*(x2.point-x1.point)/(t2-t1);
  return 1.-exp(-v.dot(v));
}
*/

/***************************************************************************************
 *  Type Functions
 **************************************************************************************/

// defines the dominant type:
//  !!! 
//  !!!  it is strongly recommended that you check if 
//  !!!  the default fallback works for your case
//  !!!
template<typename T1, typename T2>
struct trajectory_type_promotion
{
  typedef typename T1::ValueT res_distance_type;
  typedef typename T1::TimeT res_time_type;
  typedef typename T1::StateT res_state_type;
};

template<typename T>
struct trajectory_type_promotion<T,T>
{
  typedef typename T::ValueT res_distance_type;
  typedef typename T::TimeT res_time_type;
  typedef typename T::StateT res_state_type;
};

template<>
struct trajectory_type_promotion<PointTraits,PoseTraits>
{
  typedef  double res_distance_type;
  typedef typename PointTraits::TimeT res_time_type;
  typedef typename PointTraits::StateT res_state_type;
};

template<>
struct trajectory_type_promotion<PoseTraits,PointTraits>
{
  typedef double res_distance_type;
  typedef typename PointTraits::TimeT res_time_type;
  typedef typename PointTraits::StateT res_state_type;
};


/***************************************************************************************
 *  Policy Functions
 **************************************************************************************/
template<typename T1, typename T2>
struct trajectory_policy
{
  typedef typename trajectory_type_promotion<T1,T2>::res_distance_type DistanceT;
  typedef typename trajectory_type_promotion<T1,T2>::res_time_type TimeT;
  typedef typename trajectory_type_promotion<T1,T2>::res_state_type StateT;

  inline static DistanceT distance(typename T1::StateT const& xi,
                                   typename T2::StateT const& xj)
  {
    return (xi - xj).squaredNorm();
    return (xi - xj).norm();
  }

  inline static StateT lin_inter(typename T1::StateT const& x1,
                                 typename T2::StateT const& x2,
                                 TimeT const & a)
  {
    return a*x1 + (1.-a)*x2;
  }
};

template<>
typename trajectory_policy<PoseTraits,PoseTraits>::DistanceT
trajectory_policy<PoseTraits,PoseTraits>::distance(
  PoseTraits::StateT const& xi,PoseTraits::StateT const& xj)
{
  return (xi.point - xj.point).squaredNorm();
  return (xi.point - xj.point).norm();
}

template<>
typename trajectory_policy<PointTraits,PoseTraits>::DistanceT
trajectory_policy<PointTraits,PoseTraits>::distance(
  PointTraits::StateT const& xi,PoseTraits::StateT const& xj)
{
  return (xi - xj.point).squaredNorm();
  return (xi - xj.point).norm();
}

template<>
typename trajectory_policy<PoseTraits,PointTraits>::DistanceT
trajectory_policy<PoseTraits,PointTraits>::distance(
  PoseTraits::StateT const& xi,PointTraits::StateT const& xj)
{
  return (xi.point - xj).squaredNorm();
  return (xi.point - xj).norm();
}

template<>
typename trajectory_policy<PoseTraits,PoseTraits>::StateT
trajectory_policy<PoseTraits,PoseTraits>::lin_inter(
  PoseTraits::StateT const& x1,
  PoseTraits::StateT const& x2,
  typename trajectory_policy<PoseTraits,PoseTraits>::TimeT const& a)
{
  return PoseTraits::StateT{ a*x1.point + (1.f-a)*x2.point, 
      x1.orientation.slerp(1.-a, x2.orientation) };
}

#endif
