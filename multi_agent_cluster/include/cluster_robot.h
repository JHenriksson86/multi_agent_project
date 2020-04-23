#ifndef CLUSTER_ROBOT_H
#define CLUSTER_ROBOT_H
#define _USE_MATH_DEFINES

#include <vector>
#include <cmath>
#include <random>
#include <string>
#include "eigen3/Eigen/Dense"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include <visualization_msgs/Marker.h>
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
      ros::Publisher marker_pub_;

      fuzzy::FuzzyFunctions fuzzy_;
      Navigation navigation_;
      LaserScan top_scan_;
      LaserScan bottom_scan_;
      LaserScan object_candidates_;

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

         this->marker_pub_ = nh_->advertise<visualization_msgs::Marker>("marker", 100);

         random_state_ = RandomWalkState::inactive;
         random_turn_ = 0.0;
      }

      void run()
      {
         geometry_msgs::Twist msg;
         msg.linear.x = 0.0;
         msg.angular.z = 0.0; 

         avoid(&msg);
         ROS_DEBUG("Robot linear vel = %.2f, angular vel = %.2f", msg.linear.x, msg.angular.z);

         findClosestObject();
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
         
         ROS_DEBUG("Obstacle:%.2f LeftSide:%.2f FrontLeftSide:%.2f FrontRightSide:%.2f RightSide:%.2f",
                  obstacle(linear_start, linear_full),
                  obstacleLeftSide(side_start, side_full),
                  obstacleFrontLeftSide(frontside_start, frontside_full),
                  obstacleFrontRightSide(frontside_start, frontside_full),
                  obstacleRightSide(side_start, side_full)
         );
      }

      int findClosestObject()
      {
         double distance = 1.0;
         int closest_object = -1;
         if(object_candidates_.empty())
            return -1;

         for(int i = 0; i < object_candidates_.size(); i++)
         {
            LaserScanPoint candidate(object_candidates_[i]);
            int close_objects = 0;
            for(int j = 0; j < object_candidates_.size(); j++)
            {
               if(i != j)
               {
                  if(candidate.getDistance(object_candidates_[j]) < 0.2)
                     close_objects++;
               }
            }
            if(close_objects > 2)
            {
               if(candidate.getDistance() < distance)
               {
                  distance = candidate.getDistance();
                  closest_object = i;
               }
            }
         }

         if(closest_object == -1)
            ROS_INFO_STREAM("findClosestObject: No close object found");
         else
            ROS_ERROR_STREAM("findClosestObject: Object found at distance = " << distance);
      }

      double obstacleFrontLeftSide(double slowdown_threshold, double stop_threshold)
      {
         return fuzzy_.rampUp(
            top_scan_.regionDistance(degreesToRadians(0.0), degreesToRadians(10.0)), 
            slowdown_threshold, stop_threshold
         );
      }

      double obstacleLeftSide(double slowdown_threshold, double stop_threshold)
      {
         return fuzzy_.rampUp(
            top_scan_.regionDistance(degreesToRadians(10.0), degreesToRadians(90.0)), 
            slowdown_threshold, stop_threshold
         );
      }

      double obstacleFrontRightSide(double slowdown_threshold, double stop_threshold)
      {
         return fuzzy_.rampUp(
            top_scan_.regionDistance(degreesToRadians(350.0), degreesToRadians(360.0)), 
            slowdown_threshold, stop_threshold
         );
      }

      double obstacleRightSide(double slowdown_threshold, double stop_threshold)
      {
         return fuzzy_.rampUp(
            top_scan_.regionDistance(degreesToRadians(270.0), degreesToRadians(350.0)), 
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

      void publishMarker(const LaserScan& scan)
      {
         visualization_msgs::Marker marker = createMarker();
         for(int i = 0; i < scan.size(); i++) 
         {
            if(scan[i].getDistance() > 0.0)
            {
                  geometry_msgs::Point point;
                  point.x = scan[i].getX();
                  point.y = scan[i].getY();
                  point.z = 0.0;
                  marker.points.push_back(point);
            }
         } 
         marker_pub_.publish(marker);
      }

      visualization_msgs::Marker createMarker() 
      {
         visualization_msgs::Marker marker;
         
         std::string frame_id = nh_->getNamespace() + "/base_footprint";
         frame_id.erase(0,1);
         marker.header.frame_id = frame_id;
         marker.header.stamp = ros::Time();
         marker.ns = nh_->getNamespace();
         marker.type = visualization_msgs::Marker::POINTS;
         marker.action = visualization_msgs::Marker::ADD;
         
         marker.pose.position.x = 0.0;
         marker.pose.position.y = 0.0;
         marker.pose.position.z = 0.0;
         
         marker.pose.orientation.x = 0.0;
         marker.pose.orientation.y = 0.0;
         marker.pose.orientation.z = 0.0;
         marker.pose.orientation.w = 1.0;
         
         marker.scale.x = 0.05;
         marker.scale.y = 0.05;
         marker.scale.z = 0.05;

         marker.color.a = 1.0; // Don't forget to set the alpha!
         marker.color.r = 1.0;
         marker.color.g = 1.0;
         marker.color.b = 1.0;
         
         return marker;
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
         publishMarker(object_candidates_);
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