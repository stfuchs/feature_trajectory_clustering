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
};


/***************************************************************************************
 *  Predefined Traits
 **************************************************************************************/
struct DefaultTraits
{
  typedef float ValueT;
  typedef float StateT;
  typedef float TimeT;
};

struct PointTraits : DefaultTraits
{
  typedef Eigen::Matrix<float,3,1> StateT;
};

struct PoseTraits : DefaultTraits
{
  struct StateT
  {
    Eigen::Matrix<float,3,1> point;
    Eigen::Quaternion<float> orientation;
  };
};

/***************************************************************************************
 *  Type Functions
 **************************************************************************************/

// defines the dominant type:
template<typename T1, typename T2>
struct trajectory_type_promotion {};

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
  typedef typename PointTraits::ValueT res_distance_type;
  typedef typename PointTraits::TimeT res_time_type;
  typedef typename PointTraits::StateT res_state_type;
};

template<>
struct trajectory_type_promotion<PoseTraits,PointTraits>
{
  typedef typename PointTraits::ValueT res_distance_type;
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
                                   typename T2::StateT const& xj) {
    return (xi - xj).squaredNorm();
  }

  inline static StateT lin_inter(typename T1::StateT const& x1,
                                 typename T2::StateT const& x2,
                                 TimeT const & a) {
    return a*x1 + (1.-a)*x2;
  }
};

template<>
typename trajectory_policy<PoseTraits,PoseTraits>::DistanceT
trajectory_policy<PoseTraits,PoseTraits>::distance(
  PoseTraits::StateT const& xi,PoseTraits::StateT const& xj)
{
  return (xi.point - xj.point).squaredNorm();
}

template<>
typename trajectory_policy<PointTraits,PoseTraits>::DistanceT
trajectory_policy<PointTraits,PoseTraits>::distance(
  PointTraits::StateT const& xi,PoseTraits::StateT const& xj)
{
  return (xi - xj.point).squaredNorm();
}

template<>
typename trajectory_policy<PoseTraits,PointTraits>::DistanceT
trajectory_policy<PoseTraits,PointTraits>::distance(
  PoseTraits::StateT const& xi,PointTraits::StateT const& xj)
{
  return (xi.point - xj).squaredNorm();
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
