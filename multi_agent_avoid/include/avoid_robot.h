#ifndef AVOID_ROBOT_H
#define AVOID_ROBOT_H
#define _USE_MATH_DEFINES

#include <vector>
#include <cmath>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "navigation.h"
#include "fuzzy.h"
#include "laser_scan.h"

namespace robot{
   class AvoidRobot
   {
   private:

      ros::NodeHandle* nh_;
      ros::Subscriber odom_sub_;
      ros::Subscriber laserscan_sub_;
      ros::Publisher movement_pub_;

      fuzzy::FuzzyFunctions fuzzy_;
      navigation::Navigation navigation_;
      laserscan::LaserScan laserscan_;

      const float obs_front_start = 1.5;
      const float obs_front_full = 0.8;
      const float obs_side_start = 0.7;
      const float obs_side_full = 0.4;
      

   public:
      AvoidRobot(ros::NodeHandle* node_handle, std::string odom_topic, 
         std::string laserscan_topic, std::string movement_topic)
      {
         ROS_DEBUG("AvoidRobot: Creating class instance and subscribing to topics.");
         this->nh_ = node_handle;
         ROS_DEBUG_STREAM("AvoidRobot: Subscribing to odometry topic: " << odom_topic);
         this->odom_sub_ = nh_->subscribe(odom_topic, 1000, &AvoidRobot::odomCallback, this);
         ROS_DEBUG_STREAM("AvoidRobot: Subscribing to laserscan topic: " << laserscan_topic);
         this->laserscan_sub_ = nh_->subscribe(laserscan_topic, 1000, &AvoidRobot::laserscanCallback, this);
         ROS_DEBUG_STREAM("AvoidRobot: Publishing to movement topic: " << movement_topic);
         this->movement_pub_ = nh_->advertise<geometry_msgs::Twist>(movement_topic, 1000);
      }

      void run()
      {
         geometry_msgs::Twist msg;
         msg.linear.x = 0.0;
         msg.angular.z = 0.0; 

         avoid(&msg);
         ROS_INFO("Robot linear vel = %.2f, angular vel = %.2f", msg.linear.x, msg.angular.z);

         this->movement_pub_.publish(msg);
      }

      bool ok() { return nh_->ok(); }

   private:
      void avoid(geometry_msgs::Twist* msg)
      {
         //Obstacle avoidans logic
         msg->linear.x = fuzzy::FuzzyFunctions::NOT(obstacle());
         msg->angular.z -= obstacleLeftSide(obs_side_start, obs_side_full);
         msg->angular.z -= obstacleFrontLeft(obs_front_start, obs_front_full);
         msg->angular.z += obstacleFrontRight(obs_front_start, obs_front_full);
         msg->angular.z += obstacleRightSide(obs_side_start, obs_side_full);

         ROS_DEBUG("Obstacle:%.2f LeftSide:%.2f FrontLeft:%.2f FrontRight:%.2f RightSide:%.2f",
                  obstacle(),
                  obstacleLeftSide(obs_side_start, obs_side_full),
                  obstacleFrontLeft(obs_front_start, obs_front_full),
                  obstacleFrontRight(obs_front_start, obs_front_full),
                  obstacleRightSide(obs_side_start, obs_side_full));
      }

      double obstacleLeftSide(double slowDownThreshold, double stopThreshold)
      {
         return fuzzy_.rampUp(
            laserscan_.regionDistance(M_PI / 2.0, M_PI / 18.0), 
            slowDownThreshold, stopThreshold
         );
      }

      double obstacleFrontLeft(double slowDownThreshold, double stopThreshold)
      {
         return fuzzy_.rampUp(
            laserscan_.regionDistance(3.0 * M_PI / 36.0, M_PI / 36.0), 
            slowDownThreshold, stopThreshold
         );
      }

      double obstacleFrontRight(double slowDownThreshold, double stopThreshold)
      {
         return fuzzy_.rampUp(
            laserscan_.regionDistance(-M_PI / 36.0, -3.0 * M_PI / 36.0),
            slowDownThreshold, stopThreshold
         );
      }

      double obstacleRightSide(double slowDownThreshold, double stopThreshold)
      {
         return fuzzy_.rampUp(
            laserscan_.regionDistance(-M_PI / 18.0, -M_PI / 2.0), 
            slowDownThreshold, stopThreshold
         );
      }

      double obstacle()
      {
         return fuzzy::FuzzyFunctions::OR(
            obstacleLeftSide(obs_side_start, obs_side_full),
            fuzzy::FuzzyFunctions::OR(
               obstacleRightSide(obs_side_start, obs_side_full),
               fuzzy::FuzzyFunctions::OR(
                     obstacleFrontLeft(obs_front_start, obs_front_full),
                     obstacleFrontRight(obs_front_start, obs_front_full))));
      }  

      void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) 
      {
         geometry_msgs::Pose robot_pose = msg->pose.pose;
         navigation_.updatePose(
            robot_pose.position.x,
            robot_pose.position.y,
            tf::getYaw(robot_pose.orientation)
         );

         ROS_INFO("Robot Pose: [%.2f; %.2f; %.2f]", 
            navigation_.getX(), navigation_.getY(), navigation_.getTheta()
         );
      }    

      void laserscanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
      {
         ROS_DEBUG_NAMED(
            "LaserSubscriber::laser_scan_callback",
            "Got a laser scan message with %zd range measurements",
            msg->ranges.size()
         );

         laserscan_.updateLaserscan(msg);
      }
   };

} // namespace robot

#endif