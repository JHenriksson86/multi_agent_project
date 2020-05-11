#ifndef CLUSTER_ROBOT_H
#define CLUSTER_ROBOT_H
#define _USE_MATH_DEFINES

// STL
#include <vector>
#include <cmath>
#include <random>
#include <string>

// Other libs
#include "eigen3/Eigen/Dense"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"

// Custom libs
#include "utility.h"
#include "navigation.h"
#include "fuzzy.h"
#include "laser_scan.h"
#include "multi_agent_messages/Communication.h"
#include "message.h"

namespace robot{

   class ClusterRobot
   {
   private:

      ros::NodeHandle* nh_;
      ros::Subscriber odom_sub_;
      ros::Subscriber top_scan_sub_;
      ros::Subscriber bottom_scan_sub_;
      ros::Subscriber communication_sub_;
      ros::Publisher movement_pub_;
      ros::Publisher communication_pub_;
      std::string robot_ns_;

      Navigation navigation_;
      LaserScan top_scan_;
      LaserScan bottom_scan_;
      LaserScan object_candidates_;
      MessageHandler msg_handler_;

      enum class ClusteringState { Waiting, Goto, Clustering, Reversing, Turning, Stuck };
      ClusteringState cluster_state_;
      double turn_theta_;
      Eigen::Vector2d object_coordinates_;
      Eigen::Vector2d dropoff_coordinates_;
      bool old_object_;
      bool init_;

      Eigen::Vector2d unstuck_robot_coordinates_;
      ros::Time time_stuck_;
      
   public:
      ClusterRobot(ros::NodeHandle* node_handle, std::string robot_namespace,  
         std::string odom_topic, std::string movement_topic, std::string top_scan_topic, 
         std::string bottom_scan_topic, std::string communication_topic)
         : msg_handler_(robot_namespace, RobotType::cluster)
      {
         ROS_DEBUG("ClusterRobot: Creating class instance and subscribing to topics.");
         this->nh_ = node_handle;
         ROS_DEBUG_STREAM("ClusterRobot: Subscribing to odometry topic: " << odom_topic);
         this->odom_sub_ = nh_->subscribe(odom_topic, 1000, &ClusterRobot::odomCallback, this);

         ROS_DEBUG_STREAM("ClusterRobot: Publishing to movement topic: " << movement_topic);
         this->movement_pub_ = nh_->advertise<geometry_msgs::Twist>(movement_topic, 100);
         
         ROS_DEBUG_STREAM("ClusterRobot: Subscribing to top scan topic: " << top_scan_topic);
         this->top_scan_sub_ = nh_->subscribe(top_scan_topic, 100, &ClusterRobot::topScanCallback, this);
         
         ROS_DEBUG_STREAM("ClusterRobot: Subscribing to bottom scan topic: " << bottom_scan_topic);
         this->bottom_scan_sub_ = nh_->subscribe(bottom_scan_topic, 100, &ClusterRobot::bottomScanCallback, this);

         ROS_DEBUG_STREAM("ClusterRobot: Subscribing to communication topic: " << communication_topic);
         this->communication_sub_ = nh_->subscribe(communication_topic, 100, &ClusterRobot::communicationCallback, this);

         ROS_DEBUG_STREAM("ClusterRobot: Publishing to communication topic: " << communication_topic);
         this->communication_pub_ = nh_->advertise<multi_agent_messages::Communication>(communication_topic, 100);

         robot_ns_ = robot_namespace;

         cluster_state_ = ClusteringState::Waiting;
         turn_theta_ = 0.0;

         object_coordinates_ = Eigen::Vector2d::Zero();
         old_object_ = false;
         dropoff_coordinates_ = Eigen::Vector2d::Zero();

         unstuck_robot_coordinates_ = Eigen::Vector2d::Zero();
         time_stuck_ = ros::Time::now();
         init_ = true;
      }

      void run()
      {
         geometry_msgs::Twist msg;
         msg.linear.x = 0.0;
         msg.angular.z = 0.0; 

         //clustering(&msg);
         ROS_DEBUG("%s: Robot linear vel = %.2f, angular vel = %.2f", 
            robot_ns_.c_str(), msg.linear.x, msg.angular.z);

         this->movement_pub_.publish(msg);
      }

      bool ok() { return nh_->ok(); }

   private:

      void clustering(geometry_msgs::Twist* msg)
      {

         switch(cluster_state_)
         {
            case ClusteringState::Waiting:
               ROS_INFO("%s: ClusteringState::Waiting", robot_ns_.c_str());
               break;
            case ClusteringState::Goto:
               ROS_INFO("%s: ClusteringState::Goto", robot_ns_.c_str());
               break;
            case ClusteringState::Clustering:
               ROS_INFO("%s: ClusteringState::Clustering", robot_ns_.c_str());
               break;
            case ClusteringState::Reversing:
               ROS_INFO("%s: ClusteringState::Reversing", robot_ns_.c_str());
               break;
            case ClusteringState::Turning:
               ROS_INFO("%s: ClusteringState::Turning", robot_ns_.c_str());
               break;
            case ClusteringState::Stuck:
               ROS_INFO("%s: ClusteringState::Stuck", robot_ns_.c_str());
               break;
         }
      }

      bool checkStuck(int seconds)
      {
         int time_stuck = ros::Time::now().toSec() - time_stuck_.toSec();
         double distance = navigation_.getDistanceToCoordinate(unstuck_robot_coordinates_);

         if(distance > 0.1)
         {
            unstuck_robot_coordinates_ = navigation_.getCoordinates();
            time_stuck_ = ros::Time::now();
            return false;
         }
         if(time_stuck > seconds)
         {
            ROS_WARN("Stuck: Time stuck = %d, Distance %.2f", time_stuck, distance);
            return true;
         }

         return false;
      }

      bool clusterToPoint(geometry_msgs::Twist* msg, double x, double y)
      {
         Goto::gotoAvoid(navigation_, top_scan_, msg, x, y);

         if(navigation_.getDistanceToCoordinate(x, y) < 0.20)
            return true;

         return false;
      }

      Eigen::Vector2d getObjectWorldCoordinates(int index) 
      {
         return navigation_.convertToWorldCoordinate(object_candidates_[index].getCartesianCoordinates());
      }

      void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) 
      {
         geometry_msgs::Pose robot_pose = msg->pose.pose;
         navigation_.updatePose(
            robot_pose.position.x,
            robot_pose.position.y,
            tf::getYaw(robot_pose.orientation)
         );

         ROS_DEBUG("Robot Pose: [%.2f; %.2f; %.2f]", 
            navigation_.getX(), navigation_.getY(), navigation_.getTheta()
         );
      }    

      void topScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
      {
         ROS_DEBUG_NAMED(
            "ClusterRobot::topScanCallback",
            "Got a laser scan message with %zd range measurements",
            msg->ranges.size()
         );

         top_scan_.updateLaserscan(msg);

         object_candidates_ = bottom_scan_.subtractLaserScan(top_scan_);
      }

      void bottomScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
      {
         ROS_DEBUG_NAMED(
            "ClusterRobot::bottomScanCallback",
            "Got a laser scan message with %zd range measurements",
            msg->ranges.size()
         );

         bottom_scan_.updateLaserscan(msg);
      }

      void communicationCallback(const multi_agent_messages::Communication::ConstPtr &msg)
      {
         msg_handler_.addMessage(msg);
      }

   };

} // namespace robot

#endif