/**
 * @file   trajctory_visualization_node.cpp
 * @author Steffen Fuchs <steffenfuchs@samson.informatik.uni-stuttgart.de>
 * @date   Thu Nov  6 13:07:32 2014
 * 
 * @brief  ROS node that subscribes to various topics to
 *         turn trajectories into nav_msgs/Path messages
 *         for visualization in rviz.
 */

#include "mlr_visualization/trajectory_visualization_node.h"
#include "mlr_visualization/utils.h"

void TrajectoryVisualizationNode::init()
{
  ROS_INFO("Subscribing to ar_pose_marker");
  sub_alvar_ = nh_.subscribe("ar_pose_marker", 100, &TrajectoryVisualizationNode::callback,this);
  pub_path_ = nh_.advertise<visualization_msgs::MarkerArray>("trajectory_marker", 1);
}

void TrajectoryVisualizationNode::callback(const ar_track_alvar::AlvarMarkers& alvar_markers)
{
  unsigned int n = alvar_markers.markers.size();
  if (n==0) return;

  ROS_INFO("Received %i alvar_markers",n);
  //ROS_INFO("frame: %s, %i sec",alvar_markers.header.frame_id.c_str(),
  //  alvar_markers.header.stamp.nsec);
  visualization_msgs::MarkerArray v_markers;

  for(unsigned int i = 0; i<n; ++i)
  {
    int id = alvar_markers.markers[i].id;
    std::string frame_id = alvar_markers.markers[i].header.frame_id;
    geometry_msgs::PoseStamped pose = alvar_markers.markers[i].pose;
    geometry_msgs::Point point = pose.pose.position;
    
    if(traj_path_.count(id)==0)
    {
      Visualization::Utils::ColorRGB c;
      Visualization::Utils::generateColor(id,c);

      visualization_msgs::Marker marker;
      marker.header.frame_id = frame_id;//"camera_rgb_optical_frame";
      marker.header.stamp = ros::Time();
      marker.ns = "trajectory_line";
      marker.id = id;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = 0;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = .005;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = 1.;
      marker.color.r = float(c.r)/255.;
      marker.color.g = float(c.g)/255.;
      marker.color.b = float(c.b)/255.;
      marker.points.push_back(point);
      traj_path_.insert(std::make_pair(id,marker));

      marker.ns = "trajectory_point";
      marker.type = visualization_msgs::Marker::POINTS;
      marker.scale.x = .01;
      marker.scale.y = .01;
      marker.color.r = 1.;
      marker.color.g = 1.;
      marker.color.b = 1.;
      traj_points_.insert(std::make_pair(id,marker));
    }
    else
    {
      traj_path_[id].points.push_back(point);
      traj_points_[id].points.push_back(point);
      v_markers.markers.push_back(traj_path_[id]);
      v_markers.markers.push_back(traj_points_[id]);
    }
  }
  pub_path_.publish(v_markers);
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
