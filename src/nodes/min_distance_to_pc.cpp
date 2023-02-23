/*
*author: Abhijith Anil 
*email:  inspired.ai@gmail.com
*/
/* Explanation
* Have a better understanding about the Coordinate Frame Conventions for camera :http://www.ros.org/reps/rep-0103.html
* Now consider a point inside the point cloud and imagaine that point is formed on a XY plane where the perpendicular 
* distance from the plane to the camera is Z. 
* The perpendicular drawn from the camera to the plan hits at center of the XY plane 
* Also now we have the x and y coordinate of the point which is formed on the XY plane.
* Now X is the horizontal axis and Y is the vertical axis
*                             
*
*
*
*
*                                  X
*                    _____________________________  
*                   |                             |
*                   |                     *______________Z______________________
*                   |                             |  
*                   |             C|______________________Z_____________________|
*                  Y|              |              |                             | 
*                   |                             | 
*                   |                             | 
*                   |_____________________________|  
*
*
* C is the center of the plane which is Z meter away from the center of camera and * is the point on the plan
* Now we know Z is the perpendicular distance from the point to the camera. 
* If you need to find the  actual distance d from the point to the camera, you shloud calculate the hypotenuse hypot(pt.z, pt.x)
* Angle convention
* to the left of the camera is 0 degree and to the right is 180 degree. Like you stretch both the hands towards sides (deals with X axis). left 0-----|-----180 right (could be other way around, dont have sensor to test)
* to the  bottom of the camera is 0 degree and to the top is 108 degree. 
* angle the point made horizontally min_angle_radx=atan2(pt.z,pt.x);
* angle the point made Verticlly    min_angle_rady=atan2(pt.z, pt.y);
*/

/*
With thanks to Abhijit Anil. read his explanation above.
*/


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

using namespace::std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher min_distance_pub;
ros::Publisher angle_x_pub;

void callback(const PointCloud::ConstPtr& cloud){

  // tf2_ros::Buffer tfBuffer;
  // tf2_ros::TransformListener tfListener(tfBuffer);
  // PointCloud transformed_cloud;

  // geometry_msgs::TransformStamped transformStamped;
  // try
  // {
  //   transformStamped = tfBuffer.lookupTransform("odom", "camera_link_optical", ros::Time(0));
  //   cout << transformStamped << endl;
  //   tf2::doTransform(*cloud, transformed_cloud, transformStamped);
  // }
  // catch(tf2::TransformException &ex)
  // {
  //   std::cerr << ex.what() << '\n';
  //   return;
  // }
  // Transform the point cloud
  // pcl_ros::transformPointCloud(*cloud, transformed_cloud, transform);
  

  double minDistance=0.0;
  double min_angle_radx=0.0;
  double min_angle_rady=0.0;
  double min_angle_radz=0;
  double xX=0.0,yY=0.0,zZ=0.0;
  int count=0;
  // Angles are calculated in radians and can convert to degree by multpying it with 180/pi 
  BOOST_FOREACH (const pcl::PointXYZ& pt, cloud->points){//to iterate trough all the points in the filtered point cloud published by publisher
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
//  cout<<"Distance="<<minDistance<<"\n";
//  cout<<"Angle in Degree X axis="<<min_angle_radx*(180/3.14159265358979323846)<<"\n";
//  cout<<"Angle in Degree Y axis="<<min_angle_rady*(180/3.14159265358979323846)<<"\n";
//  cout<<"pointXcoordinate="<<xX<<"\n";
//  cout<<"pointYcoordinate="<<yY<<"\n";
//  cout<<"pointZcoordinate="<<zZ<<"\n";
// Publish the minimum distance and angle in x
std_msgs::Float64 minDistance_msg;
minDistance_msg.data = minDistance;
min_distance_pub.publish(minDistance_msg);

std_msgs::Float64 angle_x_msg;
angle_x_msg.data = min_angle_radx;
angle_x_pub.publish(angle_x_msg);
 //sleep(1);use sleep if you want to delay loop.
}

int main(int argc, char** argv)
{
  ros::init(argc, argv,"min_distance_to_pc");
  ros::NodeHandle nh;
  // Initialize publishers
  min_distance_pub = nh.advertise<std_msgs::Float64>("/min_distance_to_handle", 1);
  angle_x_pub = nh.advertise<std_msgs::Float64>("/angle_to_handle", 1);
  ros::Subscriber sub = nh.subscribe<PointCloud>("/extract_cylinder_indices/output", 1, callback);
  ros::spin();
}