#include <string>
#include "ros/ros.h"
#include "cluster_robot.h"

using namespace robot;
using std::string;

int main(int argc, char *argv[])
{

   // Init the connection with the ROS system.
   ros::init(argc, argv, "cluster_robot_node");
   ros::NodeHandle nh;

   ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
   ros::console::notifyLoggerLevelsChanged();

   string nh_namespace = nh.getNamespace();
   nh_namespace = nh_namespace.substr(1);
   ROS_DEBUG_STREAM("Nodehandle namespace: " << nh_namespace);
   
   // Loading parameters
   string odometry_topic = "";
   ros::param::get(nh_namespace + "/cluster_robot_node/odometry_topic", odometry_topic);
   ROS_DEBUG_STREAM("Odometry topic set: " << odometry_topic);

   string movement_topic = "";
   ros::param::get(nh_namespace + "/cluster_robot_node/movement_topic", movement_topic);
   ROS_DEBUG_STREAM("Movement topic set: " << movement_topic);
  
   string top_scan_topic = "";
   ros::param::get(nh_namespace + "/cluster_robot_node/top_scan_topic", top_scan_topic);
   ROS_DEBUG_STREAM("Top scanner topic set: " << top_scan_topic);

   string bottom_scan_topic = "";
   ros::param::get(nh_namespace + "/cluster_robot_node/bottom_scan_topic", bottom_scan_topic);
   ROS_DEBUG_STREAM("Bottom scanner topic set: " << bottom_scan_topic);

   string communication_topic = "";
   ros::param::get(nh_namespace + "/cluster_robot_node/communication_topic", communication_topic);
   ROS_DEBUG_STREAM("Communication topic set: " << communication_topic);

   // Creating class instance
   ClusterRobot robot(
      &nh,  nh_namespace, odometry_topic, movement_topic, 
      top_scan_topic, bottom_scan_topic, communication_topic
   );

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
