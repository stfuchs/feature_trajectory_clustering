/**
 * @file   lucas_kanade_tracker.cpp
 * @author Steffen Fuchs <steffen@steffen-ubuntu>
 * @date   Mon Nov 10 15:36:15 2014
 * 
 * @brief  
 * 
 * 
 */


#include <unordered_map>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

//#include <cv_bridge/cv_bridge.h>

#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <mlr_msgs/Point3dArray.h>
#include <mlr_msgs/Point2dArray.h>

struct TrackState
{
  //cv::Point2f p2d;
  float x;
  float y;
  Eigen::Vector3f p3d;
};

struct LK3dTrackerNode
{
  typedef typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr PointCloudPtr;
  
  LK3dTrackerNode(): id_count(0), fr_count(0)
  {
    itopic_ = "camera/depth_registered/points";
    otopic_ = "tracking/lk3d/points";
    ROS_INFO("Default input pointcloud topic is %s", itopic_.c_str());
    sub_ = nh_.subscribe(itopic_,1,&LK3dTrackerNode::callback,this);

    ROS_INFO("Default output topic is: %s", otopic_.c_str());
    pub3d_ = nh_.advertise<mlr_msgs::Point3dArray>("tracking/lk3d/points",1);
    pub2d_ = nh_.advertise<mlr_msgs::Point2dArray>("tracking/lk2d/points",1);
  }

  void callback(const PointCloudPtr& pc_msg)
  {
    std_msgs::Header header = pcl_conversions::fromPCL(pc_msg->header);
    if (header.stamp < last_header_.stamp)
    {
      ROS_INFO("Message timestamp older than previous: RESET");
      tracks.clear();
      id_count=0;
      fr_count=0;
    }
    last_header_ = header;

    // non-copy map to rgb data in point cloud:
    const char* pmsg = reinterpret_cast<const char*>(&(pc_msg->points[0]))+16;
    const cv::Mat image_map(480*640,1,CV_8UC3, const_cast<char*>(pmsg),32);
    // copy to gray image:
    cv::cvtColor(image_map, img1, cv::COLOR_BGR2GRAY);
    img1.rows = 480; img1.cols = 640; img1.step[0] = 640;

    std::vector<cv::Point2f> f0; // old features
    std::vector<cv::Point2f> f1f; // new features forward
    std::vector<cv::Point2f> f1b; // new features backward
    std::vector<int> ids;
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,20,0.03);

    typename std::unordered_map<int,TrackState>::iterator it;
    for (it=tracks.begin(); it!=tracks.end(); ++it)
    {
      ids.push_back(it->first);
      f0.push_back( { it->second.x, it->second.y} );
    }

    if (!tracks.empty())
    {
      std::vector<float> err;
      std::vector<unsigned char> s;
      cv::calcOpticalFlowPyrLK(img0,img1,f0,f1f,s,err,cv::Size(31,31),3,termcrit);
      cv::calcOpticalFlowPyrLK(img1,img0,f1f,f1b,s,err,cv::Size(31,31),3,termcrit);

      for(size_t i=0; i<f1f.size(); ++i)
      {
        cv::Point2f dw = f1f[i] - f1b[i]; // flow difference
        cv::Point2f wf = f1f[i] - f0[i];  // forward flow
        cv::Point2f wb = f1b[i] - f1f[i]; // backward flow
        float dw_sqr = dw.x*dw.x + dw.y*dw.y;
        float denom = wf.x*wf.x + wf.y*wf.y + wb.x*wb.x + wb.y*wb.y;
        if (dw_sqr > .5*denom + .5) {
          tracks.erase(ids[i]);
          continue;
        }
        Eigen::Vector3f pnew = subPixelInterpolation(f1f[i], pc_msg);
        if (pnew(2) != pnew(2)) {
          tracks.erase(ids[i]);
          continue;
        }
        if ((pnew - tracks[ids[i]].p3d).squaredNorm() > .01) {
          tracks.erase(ids[i]);
          continue;
        }
        tracks[ids[i]].x = f1f[i].x;
        tracks[ids[i]].y = f1f[i].y;
        tracks[ids[i]].p3d = pnew;
      }
      publishTrajectoryUpdates(pc_msg);
    }

    if (tracks.size() < 50 || fr_count % 60 == 0)
    {
      //std::vector<unsigned char> mask(640*box_scale*480*box_scale, 1);
      cv::Mat mask(480*box_scale,640*box_scale,CV_8UC1, 1);
      std::vector<cv::Point2f> f;
      std::vector<int> ids_to_delete;
      cv::goodFeaturesToTrack(img1,f, 100, 0.01, 10, cv::Mat(), 3, false, 0.04);
      cv::cornerSubPix(img1, f, cv::Size(10,10), cv::Size(-1,-1), termcrit);
      for(it=tracks.begin(); it!=tracks.end(); ++it)
      {
        size_t yi = round(it->second.y)*box_scale;
        size_t xi = round(it->second.x)*box_scale;
        if (mask.at<unsigned char>(yi, xi) == 0)
          ids_to_delete.push_back(it->first);
        else
          mask.at<unsigned char>(yi, xi) = 0;
      }
      
      for(auto id_it=ids_to_delete.begin(); id_it!=ids_to_delete.end(); ++id_it)
      {
        tracks.erase(*id_it);
      }
      
      size_t inits=0;
      for(size_t i=0; i<f.size(); ++i)
      {
        size_t yi = f[i].y*box_scale;
        size_t xi = f[i].x*box_scale;
        if (mask.at<unsigned char>(yi,xi) == 1)
        {
          Eigen::Vector3f p = subPixelInterpolation(f[i], pc_msg);
          if (p(2) == p(2))
          {
            tracks[id_count++] = { f[i].x, f[i].y, p };
            mask.at<unsigned char>(yi,xi) = 0;
            ++inits;
          }
        }
      }
      ROS_INFO("Initialized new tracks: %zu", inits);
    }
    ++fr_count;
    cv::swap(img1, img0);
  }
  
  inline size_t boxIndex(cv::Point2f const& p) {
    // idx = x/s + y/s*width/s = (x + y*stride)*scale
    return (round(p.x) + round(p.y)*box_stride)*box_scale;
  }

  void publishTrajectoryUpdates(const PointCloudPtr& pc)
  {
    mlr_msgs::Point3dArray msg3d;
    mlr_msgs::Point2dArray msg2d;
    msg3d.header = last_header_;
    msg2d.header = last_header_;
    msg2d.scale_x = 1./float(pc->width);
    msg2d.scale_y = 1./float(pc->height);
    typename std::unordered_map<int,TrackState>::iterator it;
    for(it=tracks.begin(); it!=tracks.end(); ++it)
    {
      msg2d.x.push_back(it->second.x);
      msg2d.y.push_back(it->second.y);
      msg2d.ids.push_back(it->first);
      
      msg3d.x.push_back(it->second.p3d(0));
      msg3d.y.push_back(it->second.p3d(1));
      msg3d.z.push_back(it->second.p3d(2));
      msg3d.ids.push_back(it->first);
    }
    if(!msg3d.ids.empty())
    {
      pub2d_.publish(msg2d);
      pub3d_.publish(msg3d);
    }
    else
    {
      ROS_INFO("no points tracked");
    }
  }

  Eigen::Vector3f subPixelInterpolation(
    const cv::Point2f& feature, const PointCloudPtr& pc)
  {
    size_t idx = floor(feature.x) + floor(feature.y)*pc->width;
    float s = feature.x - floor(feature.x);
    float t = feature.y - floor(feature.y);
    Eigen::Vector3f p1, p2;
    Eigen::Vector3f p11 = (*pc)[idx].getVector3fMap();
    Eigen::Vector3f p12 = (*pc)[idx+1].getVector3fMap();
    Eigen::Vector3f p21 = (*pc)[idx+pc->width].getVector3fMap();
    Eigen::Vector3f p22 = (*pc)[idx+pc->width+1].getVector3fMap();

    if (p11(2) == p11(2) && p12(2) == p12(2))
      p1 = s*p12 + (1.-s)*p11;
    else if (p11(2) == p11(2))
      p1 = p11;
    else
      p1 = p12;

    if (p21(2) == p21(2) && p22(2) == p22(2))
      p2 = s*p22 + (1.-s)*p21;
    else if (p21(2) == p21(2))
      p2 = p21;
    else
      p2 = p22;

    if (p1(2) == p1(2) && p2(2) == p2(2))
      return t*p2 + (1.-t)*p1;
    else if (p1(2) == p1(2))
      return p1;
    else
      return p2;
  }

  std::string itopic_;
  std::string otopic_;

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub2d_;
  ros::Publisher pub3d_;

  int id_count;
  int fr_count;
  float box_scale = 1./16.;
  float box_stride = 640./16.;
  cv::Mat img0; // old gray image
  cv::Mat img1; // new gray image
  std::unordered_map<int,TrackState> tracks;
  
  std_msgs::Header last_header_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lk3d_tracker");
  LK3dTrackerNode n;
  ros::Rate r(30); // hz
  while (n.nh_.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
