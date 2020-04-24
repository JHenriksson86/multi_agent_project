#ifndef UTILITY_H
#define UTILITY_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "fuzzy.h"
#include "navigation.h"
#include "laser_scan.h"

namespace robot
{

double degreesToRadians(double degree){ return (degree * M_PI) / 180.0; }

struct Avoidance
{
   static void avoid(const LaserScan& scan, geometry_msgs::Twist *msg)
   {
      const double frontside_start = 1.2;
      const double frontside_full = 0.4;
      const double side_start = 0.4;
      const double side_full = 0.1;
      const double linear_start = 1.2;
      const double linear_full = 0.4;

      double obstacle_left = obstacleLeftSide(scan, side_start, side_full);
      double obstacle_front_left = obstacleFrontLeftSide(scan, frontside_start, frontside_full);
      double obs = obstacle(scan, linear_start, linear_full);
      double obstacle_front_right = obstacleFrontRightSide(scan, frontside_start, frontside_full);
      double obstacle_right = obstacleRightSide(scan, side_start, side_full);

      //Obstacle avoidans logic
      msg->linear.x = 0.5 * FuzzyFunctions::NOT(obs);

      if ((obstacle_front_left+obstacle_left) > (obstacle_front_right+obstacle_right))
      {
         msg->angular.z = -FuzzyFunctions::OR(obstacle_front_left, obstacle_left);
      }
      else
      {
         msg->angular.z = FuzzyFunctions::OR(obstacle_front_right, obstacle_right);
      }

      ROS_DEBUG("Obstacle:%.2f LeftSide:%.2f FrontLeftSide:%.2f FrontRightSide:%.2f RightSide:%.2f",
         obs, obstacle_left, obstacle_front_left, obstacle_front_right, obstacle_right
      );
   }

   static double obstacleFrontLeftSide(const LaserScan& scan, 
      double slowdown_threshold, double stop_threshold)
   {
      return FuzzyFunctions::rampUp(
         scan.regionDistance(degreesToRadians(0.0), degreesToRadians(10.0)), 
         slowdown_threshold, stop_threshold
      ); 
   }

   static double obstacleLeftSide(const LaserScan& scan, 
      double slowdown_threshold, double stop_threshold)
   {
      return FuzzyFunctions::rampUp(
         scan.regionDistance(degreesToRadians(10.0), degreesToRadians(90.0)), 
         slowdown_threshold, stop_threshold
      );
   }

   static double obstacleFrontRightSide(const LaserScan& scan, 
      double slowdown_threshold, double stop_threshold)
   {
      return FuzzyFunctions::rampUp(
         scan.regionDistance(degreesToRadians(350.0), degreesToRadians(360.0)), 
         slowdown_threshold, stop_threshold
      );
   }

   static double obstacleRightSide(const LaserScan& scan, 
      double slowdown_threshold, double stop_threshold)
   {
      return FuzzyFunctions::rampUp(
         scan.regionDistance(degreesToRadians(270.0), degreesToRadians(350.0)), 
         slowdown_threshold, stop_threshold
      );
   }

   static double obstacle(const LaserScan& scan, 
      double slowdown_threshold, double stop_threshold)
   {
      return FuzzyFunctions::OR3(
         obstacleFrontLeftSide(scan, slowdown_threshold, stop_threshold),
         obstacleFrontRightSide(scan, slowdown_threshold, stop_threshold),
         FuzzyFunctions::OR(
            obstacleLeftSide(scan, 0.25*slowdown_threshold, 0.25*stop_threshold),
            obstacleRightSide(scan, 0.25*slowdown_threshold, 0.25*stop_threshold)
         )
      );
   }
};

struct Goto
{
   static void gotoAvoid(const Navigation& nav, const LaserScan& scan, 
      geometry_msgs::Twist* msg, double x, double y)
   {
      //Post estimated error angles
      ROS_DEBUG("Error distance = %.2f, angle = %.2f", 
         nav.getDistanceToCoordinate(x, y), 
         nav.getAngleToCoordinate(x, y)
      );

      const double frontside_start = 1.2;
      const double frontside_full = 0.4;
      const double side_start = 0.4;
      const double side_full = 0.1;
      const double linear_start = 1.2;
      const double linear_full = 0.4;

      double obstacle_left = Avoidance::obstacleLeftSide(scan, side_start, side_full);
      double obstacle_front_left = Avoidance::obstacleFrontLeftSide(scan, frontside_start, frontside_full);
      double obs = Avoidance::obstacle(scan, linear_start, linear_full);
      double obstacle_front_right = Avoidance::obstacleFrontRightSide(scan, frontside_start, frontside_full);
      double obstacle_right = Avoidance::obstacleRightSide(scan, side_start, side_full);

      //Navigation logic
      msg->angular.z += FuzzyFunctions::AND(
         positionLeft(nav.getAngleToCoordinate(x, y), degreesToRadians(50.0), degreesToRadians(3.0)),
         FuzzyFunctions::NOT(obs)
      );

      msg->angular.z -= FuzzyFunctions::AND(
         positionRight(nav.getAngleToCoordinate(x, y), -degreesToRadians(50.0), -degreesToRadians(3.0)),
         FuzzyFunctions::NOT(obs)
      );

      msg->linear.x = 0.5 * FuzzyFunctions::AND(
         FuzzyFunctions::AND(
            FuzzyFunctions::NOT(positionHere(nav.getDistanceToCoordinate(x, y), 0.4, 0.12)),
            positionAhead(nav.getAngleToCoordinate(x, y), degreesToRadians(55.0), degreesToRadians(10.0))
         ),
         FuzzyFunctions::NOT(obs)
      );

      //Obstacle avoidans logic
      if ((obstacle_front_left+obstacle_left) > (obstacle_front_right+obstacle_right))
      {
         msg->angular.z -= FuzzyFunctions::OR(obstacle_front_left, obstacle_left);
      }
      else
      {
         msg->angular.z += FuzzyFunctions::OR(obstacle_front_right, obstacle_right);
      }

   }

   static double positionLeft(double angle, double slowdown_threshold, double stop_threshold)
   {
      return FuzzyFunctions::rampDown(angle, slowdown_threshold, stop_threshold);
   }

   static double positionRight(double angle, double slowdown_threshold, double stop_threshold)
   {
      return FuzzyFunctions::rampUp(angle, stop_threshold, slowdown_threshold);
   }

   static double positionAhead(double angle, double slowdown_threshold, double stop_threshold)
   {
      return FuzzyFunctions::AND(
         FuzzyFunctions::NOT(positionLeft(angle, slowdown_threshold, stop_threshold)), 
         FuzzyFunctions::NOT(positionRight(angle, -slowdown_threshold, -stop_threshold))
      );
   }

   static double positionHere(double distance, double slowdown_threshold, double stop_threshold)
   {
      return FuzzyFunctions::rampUp(distance, slowdown_threshold, stop_threshold);
   }
};

/*
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
*/

}; // namespace robot

#endif