#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <cmath>
#include "eigen3/Eigen/Dense"

namespace robot{

   class Pose
   {
      private:
      Eigen::Vector3d pose_;

      public:
      Pose(double x = 0.0, double y = 0.0, double theta = 0.0)
      {
         setPose(x, y, theta);
      }

      void setPose(double x, double y, double theta)
      {
         pose_[0] = x;
         pose_[1] = y;
         pose_[2] = theta;
      }

      double getX() const { return pose_[0]; }

      void setX(double x) { pose_[0] = x; }

      double getY() const { return pose_[1]; }

      void setY(double y) { pose_[1] = y; }

      double getTheta() const { return pose_[2]; }

      void setTheta(double theta) { pose_[2] = theta; }

      Eigen::Vector3d getPoseVector() const 
      {
         return pose_;
      }

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

      Eigen::Vector3d getPoseVector() const
      {
         return pose_.getPoseVector();
      }

      Eigen::Vector2d convertToWorldCoordinate(const Eigen::Vector2d& coordinate) const
      {
         Eigen::Matrix2d rot_matrix;
         rot_matrix << std::cos(pose_.getTheta()), -std::sin(pose_.getTheta()),
            std::sin(pose_.getTheta()), std::cos(pose_.getTheta());
         Eigen::Vector2d trans_vec(pose_.getX(), pose_.getY());

         return rot_matrix * coordinate + trans_vec;
      }

      double getThetaSum(double angle) const { return addAngles(pose_.getTheta(), angle); }

      double getThetaDiff(double angle) const { return subtractAngles(pose_.getTheta(), angle); }

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

      double addAngles(double angle1, double angle2) const 
      {
         double angle = angle1 + angle2;
         
         if(angle > M_PI)
            return angle - 2.0 * M_PI;
         else if(angle < -M_PI)
            return angle + 2.0 * M_PI;
         return angle;
      }
   };

   
}


#endif