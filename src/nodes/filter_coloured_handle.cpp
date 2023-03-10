#include <pcl/filters/conditional_removal.h> //and the other usuals
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
#include <stdio.h>
#include <boost/foreach.hpp>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PointStamped.h>
using namespace::std;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
ros::Publisher filtered_pc_pub;
ros::Publisher min_distance_pub;
ros::Publisher angle_x_pub;
ros::Publisher handle_centroid_pub;

void callback(const PointCloud::ConstPtr& rgb_cloud){
  
  //  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCLPointCloud2 cloud_filtered_ros;

  pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter;

  pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr
      red_condition(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::GT, 100));
  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
  color_cond->addComparison (red_condition);
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, 10))); 
  color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, 10)));

  // Build the filter
  color_filter.setInputCloud(rgb_cloud);
  color_filter.setCondition (color_cond);
  color_filter.filter(*cloud_filtered);
  //  cout << cloud_filtered->is_dense;
  filtered_pc_pub.publish(cloud_filtered);
  double minDistance=0.0;
    double min_angle_radx=0.0;
    double min_angle_rady=0.0;
    double min_angle_radz=0;
    double xX=0.0,yY=0.0,zZ=0.0;
    int count=0;
    // Angles are calculated in radians and can convert to degree by multpying it with 180/pi 
    BOOST_FOREACH (const pcl::PointXYZRGB& pt, cloud_filtered->points){//to iterate trough all the points in the filtered point cloud published by publisher
      if(atan2(pt.z, pt.y)*(180/3.14159265358979323846)>0){// atan2(z,y)= arctan(z/y) if z>0;
        // truncating points with less that 80 degree vertical angle
        // because the point formed could be ground. 
          if(count==0){
          // initializing the first point read as minimum distance point
          minDistance=hypot(pt.z, pt.x);
          min_angle_radx=atan2(pt.z,pt.x);
          min_angle_rady=atan2(pt.z, pt.y);
          xX=pt.x;
          yY=pt.y;
          zZ=pt.z;
          count++;
          // cout<< "distance has been updated" << endl;
          }
        else if(hypot(pt.z, pt.x)<minDistance){
              // keep updating the minimum Distant point
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
    if (minDistance == 0) {
      minDistance = 20;
    }

    geometry_msgs::PointStamped handle_centroid;
    handle_centroid.header.frame_id = rgb_cloud->header.frame_id;
    handle_centroid.header.stamp = ros::Time(0);
    handle_centroid.point.x = xX;
    handle_centroid.point.y = yY;
    handle_centroid.point.z = zZ;
    handle_centroid_pub.publish(handle_centroid);


    std_msgs::Float32 min_distance_msg;
    min_distance_msg.data = minDistance;
    min_distance_pub.publish(min_distance_msg);

    std_msgs::Float32 angle_x_msg;
    angle_x_msg.data = min_angle_radx;
    angle_x_pub.publish(angle_x_msg);


  //  int num_points = cloud_filtered->width * cloud_filtered->height;
    // cout << num_points << " " << minDistance << " " << min_angle_radx << " " << endl;
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv,"coloured_handle_filter");
  ros::NodeHandle nh;
  // Initialize publishers
//   min_distance_pub = nh.advertise<std_msgs::Float64>("/min_distance_to_handle", 1);
//   angle_x_pub = nh.advertise<std_msgs::Float64>("/angle_to_handle", 1);
  ros::Subscriber sub = nh.subscribe<PointCloud>("/camera/depth/points", 1, callback);
  filtered_pc_pub = nh.advertise<pcl::PCLPointCloud2> ("handle_filterd_point_cloud", 1);
  min_distance_pub = nh.advertise<std_msgs::Float32>("/min_distance_to_handle", 1);
  angle_x_pub = nh.advertise<std_msgs::Float32>("/angle_to_handle", 1);
  handle_centroid_pub = nh.advertise<geometry_msgs::PointStamped>("/handle_centroid", 1);
  ros::spin();
}

