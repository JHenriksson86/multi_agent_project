#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <cmath>

namespace navigation{

   class Pose
   {
      private:
      double x_;
      double y_;
      double theta_;

      public:
      Pose(double x = 0.0, double y = 0.0, double theta = 0.0)
      {
         setPose(x, y, theta);
      }

      void setPose(double x, double y, double theta)
      {
         this->x_ = x;
         this->y_ = y;
         this->theta_ = theta;
      }

      double getX() const { return x_; }

      void setX(double x) { this->x_ = x; }

      double getY() const { return y_; }

      void setY(double y) { this->y_ = y; }

      double getTheta() const { return theta_; }

      void setTheta(double theta) { this->theta_ = theta; }
   };

   class Navigation {
      private:
      Pose pose_;

      public:
      Navigation(){}

      Navigation(const Pose& pose)
      {
         this->pose_ = pose;
      }

      ~Navigation() {}

      void updatePose(const Pose& pose)
      {
         this->pose_ = pose;
      }

      void updatePose(double x, double y, double theta)
      {
         this->pose_.setPose(x, y, theta);
      }

      double getX() const { return pose_.getX(); }

      double getY() const { return pose_.getY(); }

      double getTheta() const { return pose_.getTheta(); }

      double getDistanceToCoordinate(double x, double y) const 
      {
         double robotX = x - this->pose_.getX();
         double robotY = y - this->pose_.getY();

         return std::sqrt(std::pow(robotX, 2) + std::pow(robotY, 2));
      }

      double getAngleToCoordinate(double x, double y) const 
      {
         double robotX = x - this->pose_.getX();
         double robotY = y - this->pose_.getY();
         double pointAngle = std::atan2(robotY, robotX);

         return subtractAngles(pointAngle, pose_.getTheta());
      }

      private:
      double subtractAngles(double angle1, double angle2) const 
      {
         double angle = angle1 - angle2;
         
         if(angle > M_PI)
            return angle - 2.0 * M_PI;
         else if(angle < -M_PI)
            return angle + 2.0 * M_PI;
         return angle;
      }
   };

   
}


#endif