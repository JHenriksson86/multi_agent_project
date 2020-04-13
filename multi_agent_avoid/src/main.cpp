#include <string>
#include "ros/ros.h"
#include "avoid_robot.h"

using namespace robot;
using std::string;

int main(int argc, char *argv[])
{

   // Init the connection with the ROS system.
   ros::init(argc, argv, "multi_agent_avoid_node");
   ros::NodeHandle nh;

   ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
   ros::console::notifyLoggerLevelsChanged();

   string nh_namespace = nh.getNamespace();
   ROS_DEBUG_STREAM("Nodehandle namespace: " << nh_namespace);
   
   // Loading parameters
   string odometry_topic = "";
   ros::param::get(nh_namespace + "/multi_agent_avoid_node/odometry_topic", odometry_topic);
   ROS_DEBUG_STREAM("Odometry topic set: " << odometry_topic);
  
   string laserscan_topic = "";
   ros::param::get(nh_namespace + "/multi_agent_avoid_node/laserscan_topic", laserscan_topic);
   ROS_DEBUG_STREAM("Laserscan topic set: " << laserscan_topic);

   string movement_topic = "";
   ros::param::get(nh_namespace + "/multi_agent_avoid_node/movement_topic", movement_topic);
   ROS_DEBUG_STREAM("Movement topic set: " << movement_topic);

   // Creating class instrance
   AvoidRobot robot(&nh, odometry_topic, laserscan_topic, movement_topic);

   // ROS loop
   ros::Rate loop_rate(10);
   while (robot.ok())
   {
      robot.run();

      ros::spinOnce();
      loop_rate.sleep();
   }
   return 0;
}
