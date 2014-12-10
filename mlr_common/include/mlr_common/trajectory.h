/**
 * @file   trajectory.h
 * @author Steffen Fuchs <st.fuchs.tr@gmail.com>
 * @date   Tue Dec  9 16:31:09 2014
 * 
 * @brief  
 * 
 * 
 */

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <deque>
#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "mlr_common/traits.h"

template<typename TraitsT>
class Trajectory
{
public:
  typedef typename TraitsT::StateT StateT;
  typedef typename TraitsT::ValueT ValueT;
  typedef typename TraitsT::TimeT TimeT;

  typedef std::deque<StateT>::const_iterator StateConstPtr;
  typedef std::deque<StateT>::iterator StatePtr;
  typedef std::deque<TimeT>::const_iterator TimeConstIter;
  typedef std::deque<TimeT>::iterator TimeIter;

public:
  Trajectory() 
  { }


  /** 
   * Assumes State x and Time t are valid inserts both.
   * Removes the oldest elements until the constraints defined by TraitsT are met.
   * 
   * @param x the new state
   * @param t the new timestamp
   * 
   * @return the number of elements that had been removed
   */
  unsigned int update(const StateT& x, const TimeT& t)
  {
    timestamps_.push_front(t);
    states_.push_front(x);

    unsigned int n_removed = 0;
    while( ((t-timestamps_.back()) > TraitsT::timespan || timestamps_.size() > TraitsT::n_max)
           && timestamps_.size() > TraitsT::n_min )
    {
      timestamps_.pop_back();
      state_.pop_back();
      ++n_removed;
    }
    return n_removed;
  }

  void getStates(const TimeT& t, std::vector<StateConstPtr>& states)
  {
    TimeIter t_it = timestamps_.begin();
    StateConstPtr s = states_.begin();
    while(*t_it >= t && t != timestamps_.end())
    {
      states.push_back(s++);
      ++t_it;
    }
  }

private:
  TrajectoryId id_;
  std::deque<TimeT> timestamps_;
  std::deque<StateT> states_;
};

class TrajectoryManagement
{

private:
  std::unordered_map<TrajectoryId,Trajectory> trajectory_map_;
  
};

#endif
