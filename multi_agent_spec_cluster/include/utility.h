#ifndef UTILITY_H
#define UTILITY_H

#include <random>
#include <chrono>

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

   static double obstacleAhead(const LaserScan& scan, 
      double slowdown_threshold, double stop_threshold)
   {
      return FuzzyFunctions::rampUp(
         FuzzyFunctions::OR(
            scan.regionDistance(degreesToRadians(0.0), degreesToRadians(45.0)),
            scan.regionDistance(degreesToRadians(315.0), degreesToRadians(360.0))
         ), 
         slowdown_threshold, 
         stop_threshold
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
            positionAhead(nav.getAngleToCoordinate(x, y), degreesToRadians(30.0), degreesToRadians(10.0))
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

enum class RandomWalkState { forward, turning };

class RandomWalk
{
   private:
   double random_turn_;
   RandomWalkState state_;
   Navigation* navigation_;
   LaserScan* scan_;

   public:

   RandomWalk(Navigation* navigation, LaserScan* scan)
   {
      this->navigation_ = navigation;
      this->scan_ = scan;
      this->random_turn_ = 0.0;
      this->state_ = RandomWalkState::forward;
   }

   const RandomWalkState& getState() const { return state_; }

   void run(geometry_msgs::Twist* msg)
   {
      double obstacle_right = scan_->regionDistance(degreesToRadians(0.0), degreesToRadians(45.0));
      double obstacle_left = scan_->regionDistance(degreesToRadians(315.0), degreesToRadians(360.0));
      ROS_DEBUG("RandomWalk: obstacle left = %.2f, obstacle right = %.2f", obstacle_left, obstacle_right);

      switch(state_)
      {
         case RandomWalkState::forward:
            ROS_DEBUG("RandomWalk: state = forward");
            if(std::fmin(obstacle_left, obstacle_right) < 0.5)
            {
               ROS_DEBUG("RandomWalk: obstacle detected changing state.");
               unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
               std::default_random_engine generator(seed);
               std::uniform_real_distribution<double> distribution(degreesToRadians(45.0),degreesToRadians(90.0));
               double random_number = distribution(generator);
               ROS_DEBUG("RandomWalk: generated random number = %.2f.", random_number);

               if(obstacle_left <= obstacle_right)
               {
                  random_turn_ = navigation_->getThetaSum(random_number);
               }
               else
               {
                  random_turn_ = navigation_->getThetaDiff(random_number);
               }
               
               state_ = RandomWalkState::turning;
               msg->linear.x = 0.0;
            }
            else
            {
               msg->linear.x = 0.5 * FuzzyFunctions::NOT(Avoidance::obstacleAhead(*scan_, 0.7, 0.45));
            }
            break;
         case RandomWalkState::turning:
            ROS_DEBUG("RandomWalk: state = turning");
            double angle_difference = random_turn_ - navigation_->getTheta();
            ROS_DEBUG("RandomWalk: random avoid turn angle = %.2f, difference = %.2f", 
               random_turn_, angle_difference);

            msg->angular.z += 
               Goto::positionLeft(angle_difference, degreesToRadians(20.0), degreesToRadians(1.0));

            msg->angular.z -= 
               Goto::positionRight(angle_difference, -degreesToRadians(20.0), -degreesToRadians(1.0));

            if(std::fabs(angle_difference) < degreesToRadians(5.0))
            {
               state_ = RandomWalkState::forward;
            }
            break;
      }
   }
};

}; // namespace robot

#endif