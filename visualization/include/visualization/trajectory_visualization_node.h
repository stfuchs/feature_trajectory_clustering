/**
 * @file   trajectory_visualization_node.h
 * @author Steffen Fuchs <st.fuchs.tr@gmail.com>
 * @date   Thu Nov  6 12:48:05 2014
 * 
 * @brief  ROS node that subscribes to various topics to
 *         turn trajectories into nav_msgs/Path messages
 *         for visualization in rviz.
 */

#ifndef TRAJECTORY_VISUALIZATION_H
#define TRAJECTORY_VISUALIZATION_H

#include <map>
#include <ros/ros.h>

// message types:
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <ar_track_alvar/AlvarMarkers.h>

class TrajectoryVisualizationNode
{
public:
  TrajectoryVisualizationNode()
    : nh_()
  { init(); }

  ~TrajectoryVisualizationNode() {}

  void init();

  void callback(const ar_track_alvar::AlvarMarkers& alvar_markers);

  inline bool ok() { return nh_.ok(); }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_alvar_;
  ros::Publisher pub_path_;
  std::map<int,visualization_msgs::Marker> traj_;
  std::map<int,std_msgs::ColorRGBA> colors_;
};

#endif
