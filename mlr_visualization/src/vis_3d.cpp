/**
 * @file   trajctory_visualization_node.cpp
 * @author Steffen Fuchs <steffenfuchs@samson.informatik.uni-stuttgart.de>
 * @date   Thu Nov  6 13:07:32 2014
 * 
 * @brief  ROS node that subscribes to various topics
 *         for visualization in rviz.
 */

#include <map>
#include <ros/ros.h>

// message types:
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <mlr_msgs/Point3dArray.h>
#include <mlr_msgs/ObjectIds.h>

#include "mlr_visualization/utils.h"


struct Vis3dNode
{
  Vis3dNode() : nh_()
  {
    topic_traj_ = "lk3d/points";
    topic_obj_  = "objects";
    ROS_INFO("Default input Point2dArray topic is: %s", topic_traj_.c_str());
    ROS_INFO("Default input ObjectIds topic is: %s", topic_obj_.c_str());
    sub_traj_ = nh_.subscribe(topic_traj_, 100, &Vis3dNode::trajectory_cb,this);
    sub_objects_ = nh_.subscribe(topic_obj_, 100, &Vis3dNode::object_cb,this);
    //sub_alvar_ = nh_.subscribe("ar_pose_marker", 100, &Vis3dNode::callback_alvar,this);

    pub_path_ = nh_.advertise<visualization_msgs::MarkerArray>("trajectory_marker", 1);
  }

  void createNewMarker(int id, const std::string frame_id,
                       const geometry_msgs::Point& point)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
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
    marker.scale.x = .002;
    //marker.scale.y = 0.1;
    //marker.scale.z = 0.1;
    marker.color.a = 1.;
    marker.color.r = 1.;
    marker.color.g = 1.;
    marker.color.b = 1.;
    marker.points.push_back(point);
    traj_path_.insert(std::make_pair(id,marker));

    marker.ns = "trajectory_point";
    marker.type = visualization_msgs::Marker::POINTS;
    marker.scale.x = .008;
    marker.scale.y = .008;
    marker.color.r = 1.;
    marker.color.g = 1.;
    marker.color.b = 1.;
    marker.color.a = .3;
    traj_points_.insert(std::make_pair(id,marker));
  }

  void trajectory_cb(const mlr_msgs::Point3dArray& update)
  {
    size_t n = update.ids.size();
    ROS_INFO("Received %i lucas kanade updates",int(n));

    if (update.header.stamp < last_header_.stamp)
    {
      ROS_INFO("Message timestamp older than previous: RESET");
      visualization_msgs::MarkerArray v_markers;
      typename std::map<int,visualization_msgs::Marker>::iterator it;
      for(it=traj_path_.begin(); it!=traj_path_.end(); ++it)
      {
        int id = it->first;
        it->second.action = visualization_msgs::Marker::DELETE;
        traj_points_[id].action = visualization_msgs::Marker::DELETE;
        v_markers.markers.push_back(it->second);
        v_markers.markers.push_back(traj_points_[id]);
      }
      pub_path_.publish(v_markers);

      traj_path_.clear();
      traj_points_.clear();
    }
    last_header_ = update.header;

    //visualization_msgs::MarkerArray v_markers;

    std::string frame_id = update.header.frame_id;
    for(size_t i=0; i<n; ++i)
    {
      int id = update.ids[i];
      geometry_msgs::Point point;
      point.x = update.x[i];
      point.y = update.y[i];
      point.z = update.z[i];
    
      if(traj_path_.count(id)==0)
      {
        createNewMarker(id,frame_id,point);
      }
      else
      {
        traj_path_[id].points.push_back(point);
        traj_points_[id].points.push_back(point);
        if(traj_path_[id].points.size() > 100)
        {
          traj_path_[id].points.erase(traj_path_[id].points.begin(),
                                      traj_path_[id].points.begin()+20);
        } 
        //v_markers.markers.push_back(traj_path_[id]);
        //v_markers.markers.push_back(traj_points_[id]);
      }
    }
    //pub_path_.publish(v_markers);
  }

  void object_cb(const mlr_msgs::ObjectIds& msg)
  {
    std::vector<int> to_delete;
    for(auto it=traj_path_.begin(); it!=traj_path_.end(); ++it)
    {
      if(it->second.action == visualization_msgs::Marker::DELETE)
      {
        to_delete.push_back(it->first);
      }
      else
      {
        it->second.action = visualization_msgs::Marker::DELETE;
        it->second.color.r = 1.;
        it->second.color.g = 1.;
        it->second.color.b = 1.;
      }
    }
    
    for(auto it=to_delete.begin(); it!=to_delete.end(); ++it)
    {
      traj_path_.erase(*it);
      traj_points_.erase(*it);
    }

    for(int i=0; i<int(msg.offsets.size())-1; ++i)
    {
      Visualization::Utils::ColorRGB c;
      int l = msg.labels[i];
      if (l<Visualization::Utils::N)
        c = Visualization::Utils::palette[l];
      else
        Visualization::Utils::generateColor(l,c);

      for(int j=msg.offsets[i]; j<msg.offsets[i+1]; ++j)
      {
        int id = msg.ids[j] >> 32;
        if(traj_path_.count(id))
        {
          traj_path_[id].color.r = float(c.r)/255.;
          traj_path_[id].color.g = float(c.g)/255.;
          traj_path_[id].color.b = float(c.b)/255.;
          traj_path_[id].action = visualization_msgs::Marker::ADD;
        }
      }
    }
    
    visualization_msgs::MarkerArray v_markers;
    for(auto it=traj_path_.begin(); it!=traj_path_.end(); ++it)
      v_markers.markers.push_back(it->second);
    pub_path_.publish(v_markers);
  }

  std::string topic_traj_;
  std::string topic_obj_;
  ros::NodeHandle nh_;
  ros::Subscriber sub_traj_;
  ros::Subscriber sub_objects_;
  ros::Publisher pub_path_;
  std::map<int,visualization_msgs::Marker> traj_path_;
  std::map<int,visualization_msgs::Marker> traj_points_;
  std_msgs::Header last_header_;
  //std::map<int,std_msgs::ColorRGBA> colors_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_visualization");
  Vis3dNode n;
  ros::Rate r(30); // hz
  while (n.nh_.ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}
