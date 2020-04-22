#ifndef CLUSTER_ROBOT_H
#define CLUSTER_ROBOT_H
#define _USE_MATH_DEFINES

#include <vector>
#include <cmath>
#include <random>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "navigation.h"
#include "fuzzy.h"
#include "laser_scan.h"

namespace robot{
   enum class RandomWalkState { inactive, active };

   class ClusterRobot
   {
   private:

      ros::NodeHandle* nh_;
      ros::Subscriber odom_sub_;
      ros::Subscriber top_scan_sub_;
      ros::Subscriber bottom_scan_sub_;
      ros::Publisher movement_pub_;

      fuzzy::FuzzyFunctions fuzzy_;
      Navigation navigation_;
      LaserScan top_scan_;
      LaserScan bottom_scan_;

      RandomWalkState random_state_;
      double random_turn_;

   public:
      ClusterRobot(ros::NodeHandle* node_handle, std::string odom_topic,
      std::string movement_topic, std::string top_scan_topic, std::string bottom_scan_topic)
      {
         ROS_DEBUG("AvoidRobot: Creating class instance and subscribing to topics.");
         this->nh_ = node_handle;
         ROS_DEBUG_STREAM("AvoidRobot: Subscribing to odometry topic: " << odom_topic);
         this->odom_sub_ = nh_->subscribe(odom_topic, 1000, &ClusterRobot::odomCallback, this);

         ROS_DEBUG_STREAM("AvoidRobot: Publishing to movement topic: " << movement_topic);
         this->movement_pub_ = nh_->advertise<geometry_msgs::Twist>(movement_topic, 100);
         
         ROS_DEBUG_STREAM("AvoidRobot: Subscribing to top scan topic: " << top_scan_topic);
         this->top_scan_sub_ = nh_->subscribe(top_scan_topic, 100, &ClusterRobot::topScanCallback, this);
         
         ROS_DEBUG_STREAM("AvoidRobot: Subscribing to bottom scan topic: " << bottom_scan_topic);
         this->bottom_scan_sub_ = nh_->subscribe(bottom_scan_topic, 100, &ClusterRobot::bottomScanCallback, this);

         random_state_ = RandomWalkState::inactive;
         random_turn_ = 0.0;
      }

      void run()
      {
         geometry_msgs::Twist msg;
         msg.linear.x = 0.0;
         msg.angular.z = 0.0; 

         randomWalk(&msg);
         ROS_INFO("Robot linear vel = %.2f, angular vel = %.2f", msg.linear.x, msg.angular.z);

         this->movement_pub_.publish(msg);
      }

      bool ok() { return nh_->ok(); }

   private:
      void randomWalk(geometry_msgs::Twist* msg)
      {
         double obstacle_left = top_scan_.regionDistance(degreesToRadians(0.0), degreesToRadians(60.0));
         double obstacle_right = top_scan_.regionDistance(degreesToRadians(360.0), degreesToRadians(300.0));
         ROS_INFO("RandomWalk: obstacle left = %.2f, obstacle right = %.2f", obstacle_left, obstacle_right);

         switch(random_state_)
         {
            case RandomWalkState::inactive:
               if(std::fmin(obstacle_left, obstacle_right) < 0.5)
               {
                  std::default_random_engine generator;
                  std::uniform_real_distribution<double> distribution(degreesToRadians(45.0),degreesToRadians(135.0));
                  if(obstacle_left > obstacle_right)
                  {
                     random_turn_ = navigation_.getThetaSum(distribution(generator));
                  }
                  else if(obstacle_right > obstacle_left)
                  {
                     random_turn_ = navigation_.getThetaDiff(distribution(generator));
                  }
                  random_state_ = RandomWalkState::active;
                  msg->linear.x = 0.0;
               }
               else
               {
                  msg->linear.x = 0.5;
               }
            break;
            case RandomWalkState::active:
               double angle_difference = random_turn_ - navigation_.getTheta();
               ROS_INFO("RandomWalk: random avoid turn angle difference = %.2f", angle_difference);

               msg->angular.z += 
                  positionLeft(angle_difference, degreesToRadians(20.0), degreesToRadians(1.0));

               msg->angular.z -= 
                  positionRight(angle_difference, -degreesToRadians(20.0), -degreesToRadians(1.0));

               if(std::fabs(angle_difference) < degreesToRadians(2.0))
               {
                  random_state_ = RandomWalkState::inactive;
               }
            break;
         }

         
         
      }

      double positionLeft(double angle, double slowDownThreshold, double stopThreshold){
         return fuzzy_.rampDown(angle, slowDownThreshold, stopThreshold);
      }

      double positionRight(double angle, double slowDownThreshold, double stopThreshold){
         return fuzzy_.rampUp(angle, stopThreshold, slowDownThreshold);
      }

      double positionAhead(double angle, double slowDownThreshold, double stopThreshold){
         return fuzzy_.OR(
            fuzzy::FuzzyFunctions::NOT(positionLeft(angle, slowDownThreshold, stopThreshold)), 
            fuzzy::FuzzyFunctions::NOT(positionRight(angle, -slowDownThreshold, -stopThreshold))
         );
      }

      double positionHere(double distance, double slowDownThreshold, double stopThreshold){
         return fuzzy_.rampUp(distance, slowDownThreshold, stopThreshold);
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

      void topScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
      {
         ROS_DEBUG_NAMED(
            "ClusterRobot::topScanCallback",
            "Got a laser scan message with %zd range measurements",
            msg->ranges.size()
         );

         top_scan_.updateLaserscan(msg);
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
   };

} // namespace robot

#endif