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
#include "object_tracking.h"
#include "multi_agent_messages/Communication.h"
#include "message.h"

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
      ScoutMessageHandler msg_handler_;
      ObjectTracking tracking_;

      enum class ScoutState { Starting, Searching, Auction, Bidding};
      ScoutState state_;
      bool start_procedure_init_;

      Eigen::Vector2d object_coordinates_;
      std::vector<Eigen::Vector2d> auctioned_objects_;

      double turn_theta_;
      Eigen::Vector2d dropoff_coordinates_;
      bool old_object_;
      bool init_;

      Eigen::Vector2d unstuck_robot_coordinates_;
      ros::Time time_stuck_;

   public:

      ScoutRobot(ros::NodeHandle* node_handle, std::string robot_namespace,
         std::string odom_topic, std::string movement_topic, std::string top_scan_topic, 
         std::string bottom_scan_topic, std::string communication_topic,
         int number_of_cluster_robots)
         : random_walk_(&navigation_, &bottom_scan_), 
         msg_handler_(robot_namespace, number_of_cluster_robots),
         tracking_(&navigation_)
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
         scoutStateStarting();
         tracking_.setClusterPoint(Eigen::Vector2d::Zero());
         
         object_coordinates_ = Eigen::Vector2d::Zero();
         old_object_ = false;
      }

      void run()
      {
         // Run the message handler
         msg_handler_.run();
         while(!msg_handler_.messageOutEmpty())
         {
            communication_pub_.publish(msg_handler_.getMessage());
         }
         handleClusteredQueue();

         // Run clustering statemachine
         geometry_msgs::Twist msg;
         msg.linear.x = 0.0;
         msg.angular.z = 0.0; 
         clustering(&msg);
         ROS_DEBUG("%s: Robot linear vel = %.2f, angular vel = %.2f", 
            robot_ns_.c_str(), msg.linear.x, msg.angular.z);

         // Publish movment
         this->movement_pub_.publish(msg);
      }

      bool ok() { return nh_->ok(); }

   private:

      void clustering(geometry_msgs::Twist* msg)
      {
         switch(state_)
         {
            case ScoutState::Starting:
               if(!start_procedure_init_)
                  start_procedure_init_ = msg_handler_.initStartProcedure();
               else if(msg_handler_.startProcedureFinished())
                  scoutStateSearching();
               break;
            case ScoutState::Searching:
               random_walk_.run(msg);
               if(random_walk_.getState() == RandomWalkState::forward)
               {
                  tracking_.update(object_candidates_);
                  //tracking_.printObjects();
                  if(tracking_.foundObject())
                  {
                     object_coordinates_ = tracking_.getBestObject();
                     scoutStateAuction(object_coordinates_[0], object_coordinates_[1]);
                  }
               }
               
               break;
            case ScoutState::Auction:
               if(msg_handler_.initAuction(object_coordinates_[0], object_coordinates_[1]))
               {
                  scoutStateBidding();
               }
               break;
            case ScoutState::Bidding:
               if(msg_handler_.auctionFinished())
               {
                  tracking_.addAuctionedObject(object_coordinates_);
                  scoutStateSearching();
               }
               break;
         }
      }

      void scoutStateStarting()
      {
         ROS_INFO("%s: ScoutState::Starting", robot_ns_.c_str());
         state_ = ScoutState::Starting;
         start_procedure_init_ = false;
      }

      void scoutStateSearching()
      {
         ROS_INFO("%s: ScoutState::Searching", robot_ns_.c_str());
         state_ = ScoutState::Searching;
      }

      void scoutStateAuction(double x, double y)
      {
         ROS_INFO("%s: ScoutState::Auction, x = %.2f y = %.2f", robot_ns_.c_str(), x, y);
         state_ = ScoutState::Auction;
      }

      void scoutStateBidding()
      {
         ROS_INFO("%s: ScoutState::Bidding", robot_ns_.c_str());
         state_ = ScoutState::Bidding;
      }

      void handleClusteredQueue()
      {
         while(msg_handler_.clusteredQueueSize() > 0)
         {
            if(tracking_.removeAuctionedObject(msg_handler_.clusteredQueueFront()))
            {
               ROS_DEBUG("%s: handleClusteredQueue() successfully removed auctioned object", robot_ns_.c_str());
            }
            else
            {
               ROS_WARN("%s: handleClusteredQueue() failed to remove auctioned object", robot_ns_.c_str());
            }
            msg_handler_.clusteredQueuePop();
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

      void communicationCallback(const multi_agent_messages::Communication::Ptr &msg)
      {
         msg_handler_.addMessage(msg);
      }
   };

} // namespace robot

#endif