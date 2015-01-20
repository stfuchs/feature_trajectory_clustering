/**
 * @file   traits.h
 * @author Steffen Fuchs <steffenfuchs@samson.informatik.uni-stuttgart.de>
 * @date   Wed Dec 10 12:32:08 2014
 * 
 * @brief  
 * 
 * 
 */

#ifndef TRACKER_TYPES_HPP
#define TRACKER_TYPES_HPP

#include <type_traits>
#include "mlr_common/trajectory.hpp"

/***************************************************************************************
 *  Lucas Kanade Tracker
 **************************************************************************************/
struct LK_Tracker : Trajectory<PointTraits,LK_Tracker>
{
  typedef Trajectory<PointTraits,LK_Tracker> Base;

  LK_Tracker(IdT const& id_) : Base(id_) {}

  static const int32_t type_id = 0;
  static const unsigned int n_min = 3;
  static const unsigned int n_max = 1000;
  static const float weight;
  static const typename Base::TimeT timespan;
};

const float LK_Tracker::weight = .5;
const typename LK_Tracker::Base::TimeT LK_Tracker::timespan = 10.;


template<typename T>
struct id_traits<T,typename std::enable_if<
                     std::is_same<T,typename LK_Tracker::IdT>::value ||
                     std::is_same<T,typename LK_Tracker::IdT const>::value>::type>
{
  typedef LK_Tracker trajectory;
};


/***************************************************************************************
 *  QR-Tag Tracker
 **************************************************************************************/
struct QR_Tracker : Trajectory<PoseTraits,QR_Tracker>
{
  typedef Trajectory<PoseTraits,QR_Tracker> Base;

  QR_Tracker(IdT const& id_) : Base(id_) {}

  static const int32_t type_id = 1;
  static const unsigned int n_min = 3;
  static const unsigned int n_max = 1000;
  static const float weight;
  static const typename Base::TimeT timespan;
};

const float QR_Tracker::weight = .8;
const typename QR_Tracker::Base::TimeT QR_Tracker::timespan = 10.;


template<typename T>
struct id_traits<T,typename std::enable_if<
                     std::is_same<T,typename QR_Tracker::IdT>::value ||
                     std::is_same<T,typename QR_Tracker::IdT const>::value>::type>
{
  typedef QR_Tracker trajectory;
};


#endif
