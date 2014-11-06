/**
 * @file   trajctory_visualization_node.cpp
 * @author Steffen Fuchs <steffenfuchs@samson.informatik.uni-stuttgart.de>
 * @date   Thu Nov  6 13:07:32 2014
 * 
 * @brief  ROS node that subscribes to various topics to
 *         turn trajectories into nav_msgs/Path messages
 *         for visualization in rviz.
 */

#include "visualization/trajectory_visualization_node.h"

void TrajectoryVisualizationNode::init()
{
  ROS_INFO("Subscribing to ar_pose_marker");
  sub_alvar_ = nh_.subscribe("ar_pose_marker", 10, &TrajectoryVisualizationNode::callback,this);
  pub_path_ = nh_.advertise<visualization_msgs::MarkerArray>("trajectory_marker", 1);
}

void TrajectoryVisualizationNode::callback(const ar_track_alvar::AlvarMarkers& alvar_markers)
{
  unsigned int n = alvar_markers.markers.size();
  if (n==0) return;

  ROS_INFO("Received %i alvar_markers",n);
  //ROS_INFO("frame: %s, %i sec",alvar_markers.header.frame_id.c_str(),
  //  alvar_markers.header.stamp.nsec);

  for(unsigned int i = 0; i<n; ++i)
  {
    int id = alvar_markers.markers[i].id;
    geometry_msgs::PoseStamped pose = alvar_markers.markers[i].pose;
    geometry_msgs::Point point = pose.pose.position;
    
    if(traj_.count(id)==0)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "camera_rgb_optical_frame";
      marker.header.stamp = ros::Time::now();
      marker.ns = "trajectory_line";
      marker.id = id;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = 1;
      marker.pose.position.y = 1;
      marker.pose.position.z = 1;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.points.push_back(point);
    }
    else
    {
      traj_[id].points.push_back(point);
    }
  }
  ROS_INFO("Header: %s",path.header.frame_id.c_str());
  path.poses = traj_[2];
  pub_path_.publish(path);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_visualization");
  TrajectoryVisualizationNode n;
  ros::Rate r(30); // hz
  while (n.ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}
