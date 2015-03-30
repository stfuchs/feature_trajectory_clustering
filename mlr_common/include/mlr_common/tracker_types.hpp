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
 *  Lucas Kanade Tracker (3D)
 **************************************************************************************/
struct LK_Tracker : Trajectory<PointTraits,LK_Tracker>
{
  typedef Trajectory<PointTraits,LK_Tracker> Base;

  LK_Tracker(IdT const& id_) : Base(id_) {}

  static const int32_t type_id = 1;
  static const unsigned int n_min = 3;
  static const unsigned int n_max = 1000;
  static typename Base::TimeT timespan;
  static typename Base::ValueT lambda;
  static typename Base::ValueT gamma;
  static typename Base::ValueT eps;
};

typename LK_Tracker::Base::TimeT LK_Tracker::timespan = 10.;
typename LK_Tracker::Base::ValueT LK_Tracker::lambda = 100.;
typename LK_Tracker::Base::ValueT LK_Tracker::gamma = 10000.;
typename LK_Tracker::Base::ValueT LK_Tracker::eps = .0001;

template<typename T>
struct id_traits<T,typename std::enable_if<
                     std::is_same<T,typename LK_Tracker::IdT>::value ||
                     std::is_same<T,typename LK_Tracker::IdT const>::value>::type>
{
  typedef LK_Tracker trajectory;
};


/***************************************************************************************
 *  Lucas Kanade Tracker (2D)
 **************************************************************************************/
struct LK2d_Tracker : Trajectory<Point2dTraits,LK2d_Tracker>
{
  typedef Trajectory<Point2dTraits,LK2d_Tracker> Base;

  LK2d_Tracker(IdT const& id_) : Base(id_) {}

  static const int32_t type_id = 0;
  static const unsigned int n_min = 3;
  static const unsigned int n_max = 1000;
  static typename Base::TimeT timespan;
  static typename Base::ValueT lambda;
  static typename Base::ValueT gamma;
  static typename Base::ValueT eps;
};

typename LK2d_Tracker::Base::TimeT LK2d_Tracker::timespan = 10.;
typename LK2d_Tracker::Base::ValueT LK2d_Tracker::lambda = 100.;
typename LK2d_Tracker::Base::ValueT LK2d_Tracker::gamma = 10000.;
typename LK2d_Tracker::Base::ValueT LK2d_Tracker::eps = .0001;

template<typename T>
struct id_traits<T,typename std::enable_if<
                     std::is_same<T,typename LK2d_Tracker::IdT>::value ||
                     std::is_same<T,typename LK2d_Tracker::IdT const>::value>::type>
{
  typedef LK2d_Tracker trajectory;
};



/***************************************************************************************
 *  QR-Tag Tracker
 **************************************************************************************/
struct QR_Tracker : Trajectory<PoseTraits,QR_Tracker>
{
  typedef Trajectory<PoseTraits,QR_Tracker> Base;

  QR_Tracker(IdT const& id_) : Base(id_) {}

  static const int32_t type_id = 2;
  static const unsigned int n_min = 3;
  static const unsigned int n_max = 1000;
  static typename Base::TimeT timespan;
  static typename Base::ValueT lambda;
  static typename Base::ValueT gamma;
  static typename Base::ValueT eps;
};

typename QR_Tracker::Base::TimeT QR_Tracker::timespan = 10.;
typename QR_Tracker::Base::ValueT QR_Tracker::lambda = 100.;
typename QR_Tracker::Base::ValueT QR_Tracker::gamma = 10000.;
typename QR_Tracker::Base::ValueT QR_Tracker::eps = .0001;


template<typename T>
struct id_traits<T,typename std::enable_if<
                     std::is_same<T,typename QR_Tracker::IdT>::value ||
                     std::is_same<T,typename QR_Tracker::IdT const>::value>::type>
{
  typedef QR_Tracker trajectory;
};


#endif
