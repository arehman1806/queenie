#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>
#include <queenie/ExtremePoints.h>
#include <stdio.h>

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher min_distance_pub;
ros::Publisher angle_x_pub;
ros::Publisher filtered_pc_pub;
ros::Publisher leftmost_point_pub;
ros::Publisher rightmost_point_pub;
ros::Publisher extreme_points_pub;

void callback(const PointCloud::ConstPtr& cloud){

  double minDistance=0.0;
  double min_angle_radx=0.0;
  double min_angle_rady=0.0;
  double min_angle_radz=0;
  geometry_msgs::PointStamped leftmost;
  leftmost.header.frame_id = cloud->header.frame_id;
  leftmost.header.stamp = ros::Time(0);
  leftmost.point.x = 100;
  geometry_msgs::PointStamped rightmost; 
  rightmost.header.frame_id = cloud->header.frame_id;
  rightmost.header.stamp = ros::Time(0);
  rightmost.point.x = -100;      // initialize with a low value
  geometry_msgs::PointStamped pointcloud_centroid;
  pointcloud_centroid.header.frame_id = cloud->header.frame_id;
  pointcloud_centroid.header.stamp = ros::Time(0);

  double xX=0.0,yY=0.0,zZ=0.0;
  int count=0;
  int count_for_centroid = 0;

  // Apply passthrough filter to remove points outside of y range [-1.0, 1.0]
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-0.0001, 10);
  pass.setFilterLimitsNegative(true);
  PointCloud::Ptr filtered(new PointCloud);
  pass.filter (*filtered);
  filtered_pc_pub.publish(filtered);

  // Angles are calculated in radians and can convert to degree by multpying it with 180/pi 
  BOOST_FOREACH (const pcl::PointXYZ& pt, filtered->points){

    if (pt.x < leftmost.point.x) {
      leftmost.point.x = pt.x;
      leftmost.point.y = pt.y;
      leftmost.point.z = pt.z;
    }
    if (pt.x > rightmost.point.x) {
      rightmost.point.x = pt.x;
      rightmost.point.y = pt.y;
      rightmost.point.z = pt.z;
    }
    pointcloud_centroid.point.x += pt.x;
    pointcloud_centroid.point.y += pt.y;
    pointcloud_centroid.point.z += pt.z;
    count_for_centroid += 1;

    if(atan2(pt.z, pt.y)*(180/3.14159265358979323846)>0){
      if(count==0){
        minDistance=hypot(pt.z, pt.x);
        min_angle_radx=atan2(pt.z,pt.x);
        min_angle_rady=atan2(pt.z, pt.y);
        xX=pt.x;
        yY=pt.y;
        zZ=pt.z;
        count++;
      }
      else if(hypot(pt.z, pt.x)<minDistance){
        minDistance=hypot(pt.z, pt.x);
        min_angle_radx=atan2(pt.z,pt.x);
        min_angle_rady=atan2(pt.z, pt.y);
        xX=pt.x;
        yY=pt.y;
        zZ=pt.z;
      }
      else{
        continue;
      }
    }
  }

  pointcloud_centroid.point.x /= count_for_centroid;
  pointcloud_centroid.point.y /= count_for_centroid;
  pointcloud_centroid.point.z /= count_for_centroid;
  queenie::ExtremePoints extreme_points;
  extreme_points.leftmost = leftmost;
  extreme_points.rightmost = rightmost;
  extreme_points.point_centroid = pointcloud_centroid;
  extreme_points_pub.publish(extreme_points);
  // leftmost_point_pub.publish(leftmost);
  // rightmost_point_pub.publish(rightmost);
  
  // cout << "leftmost: " << leftmost.x << " rightmost: " << rightmost.x << endl;
  // cout << cloud->header.frame_id << endl;
  

  // Publish the minimum distance and angle in x
  if (minDistance == 0) {
    minDistance = 10;
  }
  std_msgs::Float64 minDistance_msg;
  minDistance_msg.data = minDistance;
  min_distance_pub.publish(minDistance_msg);

  std_msgs::Float64 angle_x_msg;
  angle_x_msg.data = min_angle_radx;
  // angle_x_pub.publish(angle_x_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv,"min_distance_to_pc");
  ros::NodeHandle nh;

  // Initialize publishers
  min_distance_pub = nh.advertise<std_msgs::Float64>("/min_distance_to_object", 1);
  leftmost_point_pub = nh.advertise<geometry_msgs::Point>("/leftmost_point_camera_frame", 1);
  rightmost_point_pub = nh.advertise<geometry_msgs::Point>("/rightmost_point_camera_frame", 1);
  extreme_points_pub = nh.advertise<queenie::ExtremePoints>("/extreme_points_camera_frame", 1);
  // angle_x_pub = nh.advertise<std_msgs::Float64>("/angle_to_handle", 1);
  filtered_pc_pub = nh.advertise<pcl::PCLPointCloud2> ("filtered_pc_check", 1);

  ros::Subscriber sub = nh.subscribe<PointCloud>("camera/depth/points", 1, callback);
  ros::spin();
}
