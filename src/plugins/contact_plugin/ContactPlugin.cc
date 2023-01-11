#include "ContactPlugin.hh"
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <geometry_msgs/WrenchStamped.h>

using namespace gazebo;
using namespace std;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

ros::NodeHandle nh;
ros::Publisher chatter_pub;

/////////////////////////////////////////////////
ContactPlugin::ContactPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ContactPlugin::~ContactPlugin()
{
}

/////////////////////////////////////////////////
void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&ContactPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);

  // adding a ros publisher
  int argc = 0;
  char **argv;
  ros::init(argc, argv, "talkerpapa");
  chatter_pub = nh.advertise<std_msgs::Float64>("chatter", 1000);
//  nh.advertise<std_msgs::Float64>("chatter", 1000);
  std::cout << "plugin shouldve been loaded by now hopefully";

}

/////////////////////////////////////////////////
void ContactPlugin::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  auto xy = contacts.contact_size();
//  std::cout << "number of contacts: " << xy << "and first contact has: " << contacts.contact(1).wrench_size() << "\n";
  float z_torque = 0;
  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
      if (contacts.contact(i).collision2().compare("ground_plane::link::collision") == 0
        || contacts.contact(i).collision1().compare("ground_plane::link::collision") == 0) {
//          cout << "ignoring " << contacts.contact(i).collision2();
          continue;
      }
//      cout <<  contacts.contact(i).collision2().compare("ground_plane::link::collision");
//      cout << contacts.contact(i).collision2() << "\n";
//      cout << contacts.contact(i).collision1() << "\n";
      auto wrench_1 =  contacts.contact(i).wrench(0).body_1_wrench();
      auto wrench_2 = contacts.contact(i).wrench(0).body_2_wrench();

//      cout << wrench_1.torque().z();
//      cout << contacts.contact(i).position_size() << "\n";
//    std::cout << "Collision between[" << contacts.contact(i).collision1()
//              << "] and [" << contacts.contact(i).collision2() << "]\n";


    for (unsigned int j = 0; j < contacts.contact(i).wrench_size(); ++j)
    {
//        cout << contacts.contact(i).wrench(j).body_1_name() << "\n";
//        cout << contacts.contact(i).wrench(j).body_2_wrench().torque().z() << "\n";
        z_torque += contacts.contact(i).wrench(j).body_2_wrench().torque().z();
//      std::cout << j << "  Position:"
//                << contacts.contact(i).position(j).x() << " "
//                << contacts.contact(i).position(j).y() << " "
//                << contacts.contact(i).position(j).z() << "\n";
//      std::cout << "   Normal:"
//                << contacts.contact(i).normal(j).x() << " "
//                << contacts.contact(i).normal(j).y() << " "
//                << contacts.contact(i).normal(j).z() << "\n";
//      std::cout << "   Depth:" << contacts.contact(i).depth(j) << "\n";
    }
  }
//  cout << z_torque << "\n";
  std_msgs::Float64 z_torque_msg;
  z_torque_msg.data = z_torque;
  geometry_msgs::WrenchStamped wrench;
  std_msgs::String msg;
//  std::stringstream ss;
//  ss << "hello world ";
//  msg.data = ss.str();
  chatter_pub.publish(z_torque_msg);
}
