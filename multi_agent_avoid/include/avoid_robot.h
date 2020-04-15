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
      const float obs_side_start = 1.2;
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
         const double frontside_start = 1.2;
         const double frontside_full = 0.4;
         const double side_start = 0.4;
         const double side_full = 0.1;
         const double linear_start = 1.2;
         const double linear_full = 0.4;
         
         //Obstacle avoidans logic
         msg->linear.x = 0.5 * fuzzy::FuzzyFunctions::NOT(obstacle(linear_start, linear_full));

         double obs_left = obstacleFrontLeftSide(frontside_start, frontside_full) 
            + obstacleLeftSide(side_start, side_full);
         double obs_right = obstacleFrontRightSide(frontside_start, frontside_full)
            + obstacleRightSide(side_start, side_full);

         if(obs_left > obs_right)
         {
            msg->angular.z = -fuzzy::FuzzyFunctions::OR(
               obstacleFrontLeftSide(frontside_start, frontside_full),
               obstacleLeftSide(side_start, side_full)
            );
         }
         else
         {
            msg->angular.z = fuzzy::FuzzyFunctions::OR(
               obstacleFrontRightSide(frontside_start, frontside_full),
               obstacleRightSide(side_start, side_full)
            );
         }
         
         ROS_INFO("Obstacle:%.2f LeftSide:%.2f FrontLeftSide:%.2f FrontRightSide:%.2f RightSide:%.2f",
                  obstacle(linear_start, linear_full),
                  obstacleLeftSide(side_start, side_full),
                  obstacleFrontLeftSide(frontside_start, frontside_full),
                  obstacleFrontRightSide(frontside_start, frontside_full),
                  obstacleRightSide(side_start, side_full)
         );
      }

      double obstacleFrontLeftSide(double slowdown_threshold, double stop_threshold)
      {
         return fuzzy_.rampUp(
            laserscan_.regionDistance(degreesToRadians(0.0), degreesToRadians(10.0)), 
            slowdown_threshold, stop_threshold
         );
      }

      double obstacleLeftSide(double slowdown_threshold, double stop_threshold)
      {
         return fuzzy_.rampUp(
            laserscan_.regionDistance(degreesToRadians(10.0), degreesToRadians(90.0)), 
            slowdown_threshold, stop_threshold
         );
      }

      double obstacleFrontRightSide(double slowdown_threshold, double stop_threshold)
      {
         return fuzzy_.rampUp(
            laserscan_.regionDistance(degreesToRadians(350.0), degreesToRadians(359.0)), 
            slowdown_threshold, stop_threshold
         );
      }

      double obstacleRightSide(double slowdown_threshold, double stop_threshold)
      {
         return fuzzy_.rampUp(
            laserscan_.regionDistance(degreesToRadians(270.0), degreesToRadians(350.0)), 
            slowdown_threshold, stop_threshold
         );
      }

      double obstacle(double slowdown_threshold, double stop_threshold)
      {
         return fuzzy::FuzzyFunctions::OR3(
            obstacleFrontLeftSide(slowdown_threshold, stop_threshold),
            obstacleFrontRightSide(slowdown_threshold, stop_threshold),
            fuzzy::FuzzyFunctions::OR(
               obstacleLeftSide(0.25*slowdown_threshold, 0.25*stop_threshold),
               obstacleRightSide(0.25*slowdown_threshold, 0.25*stop_threshold)
            )
         );
      }  

      double degreesToRadians(double degree){ return (degree * M_PI) / 180.0; }

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