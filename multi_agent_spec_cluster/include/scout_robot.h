#ifndef SCOUT_ROBOT_H
#define SCOUT_ROBOT_H
#define _USE_MATH_DEFINES

// Stl
#include <vector>
#include <cmath>
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

namespace robot{
   
   class ScoutRobot
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
      RandomWalk random_walk_;

      enum class ScoutState { Searching, Auction };
      ScoutState state_;
      double turn_theta_;
      Eigen::Vector2d object_coordinates_;
      Eigen::Vector2d dropoff_coordinates_;
      bool old_object_;
      bool init_;

      Eigen::Vector2d unstuck_robot_coordinates_;
      ros::Time time_stuck_;

   public:

      ScoutRobot(ros::NodeHandle* node_handle, std::string robot_namespace,
         std::string odom_topic, std::string movement_topic, std::string top_scan_topic, 
         std::string bottom_scan_topic, std::string communication_topic)
         : random_walk_(&navigation_, &bottom_scan_)
      {
         ROS_DEBUG("ScoutRobot: Creating class instance and subscribing to topics.");
         this->nh_ = node_handle;

         ROS_DEBUG_STREAM("ScoutRobot: Subscribing to odometry topic: " << odom_topic);
         this->odom_sub_ = nh_->subscribe(odom_topic, 1000, &ScoutRobot::odomCallback, this);

         ROS_DEBUG_STREAM("ScoutRobot: Publishing to movement topic: " << movement_topic);
         this->movement_pub_ = nh_->advertise<geometry_msgs::Twist>(movement_topic, 100);
         
         ROS_DEBUG_STREAM("ScoutRobot: Subscribing to top scan topic: " << top_scan_topic);
         this->top_scan_sub_ = nh_->subscribe(top_scan_topic, 100, &ScoutRobot::topScanCallback, this);
         
         ROS_DEBUG_STREAM("ScoutRobot: Subscribing to bottom scan topic: " << bottom_scan_topic);
         this->bottom_scan_sub_ = nh_->subscribe(bottom_scan_topic, 100, &ScoutRobot::bottomScanCallback, this);

         ROS_DEBUG_STREAM("ScoutRobot: Subscribing to communication topic: " << communication_topic);
         this->communication_sub_ = nh_->subscribe(communication_topic, 100, &ScoutRobot::communicationCallback, this);

         ROS_DEBUG_STREAM("ScoutRobot: Publishing to communication topic: " << communication_topic);
         this->communication_pub_ = nh_->advertise<multi_agent_messages::Communication>(communication_topic, 100);

         robot_ns_ = robot_namespace;

         state_ = ScoutState::Searching;
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

         random_walk_.run(&msg);
         ROS_DEBUG("%s: Robot linear vel = %.2f, angular vel = %.2f", 
            robot_ns_.c_str(), msg.linear.x, msg.angular.z);

         this->movement_pub_.publish(msg);
      }

      bool ok() { return nh_->ok(); }

   private:

      void clustering(geometry_msgs::Twist* msg)
      {
         
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

      

      bool findClosestObject()
      {
         if(object_candidates_.empty())
            return false;
         
         double distance = 1.0;
         int closest_object = -1;

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
                  if(old_object_)
                  {
                     Eigen::Vector2d new_object_coordinates = getObjectWorldCoordinates(closest_object);
                     double object_distance = (object_coordinates_ - new_object_coordinates).squaredNorm();
                     if(object_distance < 0.1)
                     {
                        object_coordinates_ = new_object_coordinates;
                        return true;
                     }
                  }
               }
            }
         }

         if(closest_object != -1)
         {
            object_coordinates_ = getObjectWorldCoordinates(closest_object);
            old_object_ = true;
            return true;
         }

         old_object_ = false;
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
         ROS_DEBUG(
            "Got a laser scan message with %zd range measurements",
            msg->ranges.size()
         );

         top_scan_.updateLaserscan(msg);

         object_candidates_ = bottom_scan_.subtractLaserScan(top_scan_);
      }

      void bottomScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
      {
         ROS_DEBUG(
            "Got a laser scan message with %zd range measurements",
            msg->ranges.size()
         );

         bottom_scan_.updateLaserscan(msg);
      }

      void communicationCallback(const multi_agent_messages::Communication::ConstPtr &msg)
      {
         if(msg->receiver.compare(robot_ns_) == 0)
         {
            ROS_INFO(
               "%s:Received message\nfrom: %s\nto: %s\ntype: %s\nmessage: %s",
               robot_ns_.c_str(), msg->sender.c_str(), msg->receiver.c_str(), 
               msg->message_type.c_str(), msg->message.c_str()
            );
         }
      }
   };

} // namespace robot

#endif