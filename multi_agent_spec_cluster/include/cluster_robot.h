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
#include "utility.h"
#include "navigation.h"
#include "fuzzy.h"
#include "laser_scan.h"

namespace robot{
   enum class RandomWalkState { inactive, active };

   enum class ClusteringState { Searching, Goto, Clustering, Reversing, Turning, Stuck };

   class ClusterRobot
   {
   private:

      ros::NodeHandle* nh_;
      ros::Subscriber odom_sub_;
      ros::Subscriber top_scan_sub_;
      ros::Subscriber bottom_scan_sub_;
      ros::Publisher movement_pub_;

      Navigation navigation_;
      LaserScan top_scan_;
      LaserScan bottom_scan_;
      LaserScan object_candidates_;

      RandomWalkState random_state_;
      double random_turn_;

      ClusteringState cluster_state_;
      double turn_theta_;
      Eigen::Vector2d object_coordinates_;
      Eigen::Vector2d dropoff_coordinates_;
      bool old_object_;
      bool init_;

      Eigen::Vector2d unstuck_robot_coordinates_;
      ros::Time time_stuck_;
      
      std::string robot_ns_;

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

         cluster_state_ = ClusteringState::Searching;
         turn_theta_ = 0.0;

         object_coordinates_ = Eigen::Vector2d::Zero();
         old_object_ = false;
         dropoff_coordinates_ = Eigen::Vector2d::Zero();

         unstuck_robot_coordinates_ = Eigen::Vector2d::Zero();
         time_stuck_ = ros::Time::now();
         init_ = true;

         robot_ns_ = node_handle->getNamespace();
      }

      void run()
      {
         geometry_msgs::Twist msg;
         msg.linear.x = 0.0;
         msg.angular.z = 0.0; 

         clustering(&msg);
         ROS_DEBUG("%s: Robot linear vel = %.2f, angular vel = %.2f", 
            robot_ns_.c_str(), msg.linear.x, msg.angular.z);

         this->movement_pub_.publish(msg);
      }

      bool ok() { return nh_->ok(); }

   private:

      void clustering(geometry_msgs::Twist* msg)
      {
         bool found_object = false;
         if(checkStuck(5) && !init_)
         {
            cluster_state_ = ClusteringState::Stuck;
            dropoff_coordinates_ = navigation_.getCoordinates();
         }
         else
         {
            found_object = findClosestObject();
         }

         switch(cluster_state_)
         {
            case ClusteringState::Searching:

               ROS_INFO("%s: ClusteringState::Searching", robot_ns_.c_str());
               if(!found_object)
               {
                  //Avoidance::avoid(top_scan_, msg);
                  randomWalk(msg);
               }
               else
               {
                  cluster_state_ = ClusteringState::Goto;
                  init_ = false;
               }
            break;
            case ClusteringState::Goto:
            
               if(found_object)
               {
                  ROS_INFO("%s: ClusteringState::Goto at coordinates x = %.2f, y = %.2f", 
                     robot_ns_.c_str(), object_coordinates_[0], object_coordinates_[1]);

                  Goto::gotoAvoid(navigation_, top_scan_, msg, 
                     object_coordinates_[0], object_coordinates_[1]);

                  if(navigation_.getDistanceToCoordinate(object_coordinates_[0], object_coordinates_[1]) < 0.15)
                  {
                     cluster_state_ = ClusteringState::Clustering;
                  }
               }
               else
               {
                  cluster_state_ = ClusteringState::Searching;
               }
            break;
            case ClusteringState::Clustering:

               ROS_INFO("%s: ClusteringState::Clustering", robot_ns_.c_str());
               msg->linear.x = 0.5 * FuzzyFunctions::NOT(Avoidance::obstacle(top_scan_, 1.0, 0.25));
               msg->angular.z = 0.0;

               if(atWall(top_scan_, 0.3))
               {
                  dropoff_coordinates_ = navigation_.getCoordinates();
                  cluster_state_ = ClusteringState::Reversing;
               }
               if(navigation_.getDistanceToCoordinate(object_coordinates_[0], object_coordinates_[1]) > 0.25)
               {
                  cluster_state_ = ClusteringState::Searching;
               }
            break;
            case ClusteringState::Reversing:

               ROS_INFO("%s: ClusteringState::Reversing", robot_ns_.c_str());
               msg->linear.x = -0.5;
               msg->angular.z = 0.0;
               if(navigation_.getDistanceToCoordinate(dropoff_coordinates_) > 1.5)
               {
                  cluster_state_ = ClusteringState::Turning;
                  turn_theta_ = navigation_.getThetaSum(M_PI/2.0);
               }
            break;
            case ClusteringState::Turning:

               ROS_INFO("%s: ClusteringState::Turning", robot_ns_.c_str());
               msg->linear.x = 0.0;
               msg->angular.z = 1.0;
               if(std::fabs(navigation_.getThetaDiff(turn_theta_)) < 0.1)
               {
                  cluster_state_ = ClusteringState::Searching;
               }
            break;
            case ClusteringState::Stuck:

               ROS_INFO("%s: ClusteringState::Stuck", robot_ns_.c_str());
               randomWalk(msg);
               if(navigation_.getDistanceToCoordinate(dropoff_coordinates_) > 1.5)
               {
                  cluster_state_ = ClusteringState::Searching;
               }
            break;
         }
      }

      bool atWall(const LaserScan& scan, double threshold) const
      {
         double distance = std::fmin(
            scan.regionDistance(degreesToRadians(355.0), degreesToRadians(360.0)),
            scan.regionDistance(degreesToRadians(0.0), degreesToRadians(5.0))
         );

         if(distance < threshold)
            return true;
         return false;
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

      void randomWalk(geometry_msgs::Twist* msg)
      {
         double obstacle_left = top_scan_.regionDistance(degreesToRadians(0.0), degreesToRadians(60.0));
         double obstacle_right = top_scan_.regionDistance(degreesToRadians(360.0), degreesToRadians(300.0));
         ROS_DEBUG("RandomWalk: obstacle left = %.2f, obstacle right = %.2f", obstacle_left, obstacle_right);

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
                  Goto::positionLeft(angle_difference, degreesToRadians(20.0), degreesToRadians(1.0));

               msg->angular.z -= 
                  Goto::positionRight(angle_difference, -degreesToRadians(20.0), -degreesToRadians(1.0));

               if(std::fabs(angle_difference) < degreesToRadians(2.0))
               {
                  random_state_ = RandomWalkState::inactive;
               }
            break;
         }
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
   };

} // namespace robot

#endif