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
using namespace::std;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
ros::Publisher pub;

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
 pub.publish(cloud_filtered);
 double minDistance=0.0;
  double min_angle_radx=0.0;
  double min_angle_rady=0.0;
  double min_angle_radz=0;
  double xX=0.0,yY=0.0,zZ=0.0;
  int count=0;
  // Angles are calculated in radians and can convert to degree by multpying it with 180/pi 
  BOOST_FOREACH (const pcl::PointXYZRGB& pt, rgb_cloud->points){//to iterate trough all the points in the filtered point cloud published by publisher
    if(atan2(pt.z, pt.y)*(180/3.14159265358979323846)>80.00){// atan2(z,y)= arctan(z/y) if z>0;
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
 int num_points = cloud_filtered->width * cloud_filtered->height;
    cout << num_points << " " << minDistance << " " << min_angle_radx << " " << minDistance*minDistance*num_points << endl;
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv,"filter_color_handle");
  ros::NodeHandle nh;
  // Initialize publishers
//   min_distance_pub = nh.advertise<std_msgs::Float64>("/min_distance_to_handle", 1);
//   angle_x_pub = nh.advertise<std_msgs::Float64>("/angle_to_handle", 1);
  ros::Subscriber sub = nh.subscribe<PointCloud>("/camera/depth/points", 1, callback);
  pub = nh.advertise<pcl::PCLPointCloud2> ("test_output", 1);
  ros::spin();
}

