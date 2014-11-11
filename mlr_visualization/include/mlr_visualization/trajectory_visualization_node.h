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
#include <mlr_msgs/TrajectoryPointUpdateArray.h>

class TrajectoryVisualizationNode
{
public:
  TrajectoryVisualizationNode()
    : nh_()
  { init(); }

  ~TrajectoryVisualizationNode() {}

  void init();

  void callback_alvar(const ar_track_alvar::AlvarMarkers& alvar_markers);

  void callback_lk(const mlr_msgs::TrajectoryPointUpdateArray& update_array);

  void createNewMarker(int id, const std::string frame_id, const geometry_msgs::Point& point);

  inline bool ok() { return nh_.ok(); }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_alvar_;
  ros::Subscriber sub_lk_;
  ros::Publisher pub_path_;
  std::map<int,visualization_msgs::Marker> traj_path_;
  std::map<int,visualization_msgs::Marker> traj_points_;
  //std::map<int,std_msgs::ColorRGBA> colors_;
};

#endif
