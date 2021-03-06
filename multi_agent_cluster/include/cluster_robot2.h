#ifndef CLUSTER_ROBOT_H
#define CLUSTER_ROBOT_H
#define _USE_MATH_DEFINES

#include <vector>
#include <cmath>
#include <random>
#include <string>
#include <tuple>
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

   enum class ClusteringState { Searching, Goto, Clustering, Reversing, Turning };

   class ClusterRobot
   {
   private:

      ros::NodeHandle* nh_;
      ros::Subscriber odom_sub_;
      ros::Subscriber top_scan_sub_;
      ros::Subscriber bottom_scan_sub_;
      ros::Publisher movement_pub_;
      std::string robot_ns_;

      Navigation navigation_;
      LaserScan top_scan_;
      LaserScan bottom_scan_;
      LaserScan object_candidates_;
      std::vector<std::tuple<double,double,int> > object_count_;

      RandomWalkState random_state_;
      double random_turn_;

      ClusteringState cluster_state_;
      double turn_theta_;
      int closest;
      

      double old_x;
      double old_y;
      int DT=0;
      bool flag=true;

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
         closest = -1;

         robot_ns_ = node_handle->getNamespace();
      }

      void run()
      {
         geometry_msgs::Twist msg;
         msg.linear.x = 0.0;
         msg.angular.z = 0.0;

         clustering(&msg);
         robotStuck();

         ROS_INFO("%s: Robot linear vel = %.2f, angular vel = %.2f", 
            robot_ns_.c_str(), msg.linear.x, msg.angular.z);

         this->movement_pub_.publish(msg);
      }

      bool ok() { return nh_->ok(); }

   private:
     void clustering(geometry_msgs::Twist* msg)
      {
         int closest = findClosestObject();
         int cluster_index = findClosester();
         switch(cluster_state_)
         {
            case ClusteringState::Searching:
               ROS_INFO("%s: ClusteringState::Searching", robot_ns_.c_str());
               if(closest == -1)
               {
                  //Avoidance::avoid(top_scan_, msg);
                  randomWalk(msg);
               }
               else
               {
                  cluster_state_ = ClusteringState::Goto;
               }
               break;

               case ClusteringState::Goto:

               if(closest != -1)
               {
                  Eigen::Vector2d world_cordinates =
                  navigation_.convertToWorldCoordinate(object_candidates_[closest].getCartesianCoordinates());
                  ROS_INFO("%s: ClusteringState::Goto at coordinates x = %.2f, y = %.2f",
                     robot_ns_.c_str(), world_cordinates[0], world_cordinates[1]);

                  Goto::gotoAvoid(navigation_, top_scan_, msg,
                     world_cordinates[0], world_cordinates[1]);

                  if(navigation_.getDistanceToCoordinate(world_cordinates[0], world_cordinates[1]) < 0.15)
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
               if(cluster_index != -1)
               {
                  Eigen::Vector2d cluster_cordinates = navigation_.convertToWorldCoordinate(object_candidates_[cluster_index].getCartesianCoordinates());
                  Goto::gotoAvoid(navigation_, top_scan_, msg,cluster_cordinates[0],cluster_cordinates[0]);
                  if(navigation_.getDistanceToCoordinate(cluster_cordinates[0],cluster_cordinates[0]) < 0.20)
                  {
                     cluster_state_ = ClusteringState::Reversing;
                  }
               }
               else
               {
                  Goto::gotoAvoid(navigation_, top_scan_, msg, 0.0, 0.0);
                  if(navigation_.getDistanceToCoordinate(0.0, 0.0) < 0.20)
                  {
                     cluster_state_ = ClusteringState::Reversing;
                  }
               } 
               /*
               if(objectInGripper() == 0){
                  cluster_state_ = ClusteringState::Searching;
               }
               */
            break;

            case ClusteringState::Reversing:
               ROS_INFO("%s: ClusteringState::Reversing", robot_ns_.c_str());
               msg->linear.x = -0.5;
               msg->angular.z = 0.0;
               if(obstacleInBack()){
                  msg->angular.z = 0.2;
               }

               if(cluster_index != -1)
               {
                 Eigen::Vector2d cluster_cordinates = navigation_.convertToWorldCoordinate(object_candidates_[cluster_index].getCartesianCoordinates());
                 if(navigation_.getDistanceToCoordinate(cluster_cordinates[0],cluster_cordinates[0]) > 1.5)
                 {
                   cluster_state_ = ClusteringState::Turning;
                   turn_theta_ = navigation_.getThetaSum(M_PI/2.0);
                 }
               }else{
                 if(navigation_.getDistanceToCoordinate(0.0, 0.0) > 1.5)
                 {
                   cluster_state_ = ClusteringState::Turning;
                   turn_theta_ = navigation_.getThetaSum(M_PI/2.0);
                 }
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
         }
      }

      int findClosester()
      {
         if(object_candidates_.empty())
            return -1;

         double distance = 8.0;
         int closest_object_big = -99;
         int cluster_index = -1;
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
               }
            }

            if(close_objects > closest_object_big && closest_object != i)
            {
               cluster_index = i;
               closest_object_big = close_objects;
            }
         }

         return cluster_index;
      }

      int findClosestObject()
      {
         if(object_candidates_.empty())
            return -1;

         double distance = 8.0;
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
               }
            }
         }

         return closest_object;
      }

      void robotStuck(){
        if(flag){
          DT = 0;
          old_x = navigation_.getX();
          old_y = navigation_.getY();
          flag=false;
        }
        if(DT >= 100){
          std::cout<<"\n\n\n 10s counted \n\n\n";
          flag=true;
        }
        DT++;
      }

      int objectInGripper(){
        double object = bottom_scan_.regionDistance(degreesToRadians(0.0), degreesToRadians(180.0));
        ROS_INFO("%s and Gripper object Distance: %0.2f", robot_ns_.c_str(),object);
        if(object < 0.15)
          return 1;
        else
          return 0;
      }
      bool obstacleInBack(){
        double object = top_scan_.regionDistance(degreesToRadians(360), degreesToRadians(180.0));
        if(object < 0.2)
          return true;
        else
          return false;
      }

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
