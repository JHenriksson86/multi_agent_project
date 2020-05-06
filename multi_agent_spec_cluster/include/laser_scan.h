#ifndef LASER_SCAN_H
#define LASER_SCAN_H

#include <string>
#include <iostream>
#include <vector>
#include "eigen3/Eigen/Dense"


namespace robot{

class LaserScanPoint 
{
   private:
   Eigen::Vector2d polar_coordinates_;
   Eigen::Vector2d cart_coordinates_;

   public:
   LaserScanPoint() 
   {
      polar_coordinates_ = Eigen::Vector2d::Zero();
      cart_coordinates_ = Eigen::Vector2d::Zero();
   }

   LaserScanPoint(double distance, double angle) 
   {
      this->setPolarCoordinates(distance, angle);
   }

   LaserScanPoint(const LaserScanPoint& point)
   {
      this->polar_coordinates_ = point.polar_coordinates_;
      this->cart_coordinates_ = point.cart_coordinates_;
   }

   void setPolarCoordinates(double distance, double angle) 
   {
      polar_coordinates_ << distance, angle;
      cart_coordinates_ << distance * std::cos(angle),
            distance * std::sin(angle);
   }

   void setCartesianCoordinates(double x, double y) 
   { 
      cart_coordinates_ << x, y;
      polar_coordinates_ << std::sqrt(std::pow(x, 2) + std::pow(y, 2)),
            std::atan2(y, x);
   }

   Eigen::Vector2d getCartesianCoordinates() const
   {
      return cart_coordinates_;
   }

   double getDistance(const LaserScanPoint& point) const
   {
      return (cart_coordinates_ - point.getCartesianCoordinates()).squaredNorm();
   }

   double getDistance(const Eigen::Vector2d& point) const
   {
      return (cart_coordinates_ - point).squaredNorm();
   }
   
   double getDistance() const { return polar_coordinates_[0]; }

   double getAngle() const { return polar_coordinates_[1]; } 

   double getX() const { return cart_coordinates_[0]; }

   double getY() const { return cart_coordinates_[1]; }

   ~LaserScanPoint(){}
};

class LaserScan
{
private:
   std::vector<LaserScanPoint> points_;
   const unsigned int max_distance_;

public:
   LaserScan(unsigned int max_distance = 8.0, unsigned int points = 360) 
      : max_distance_(max_distance)
   {
      points_.reserve(points);
   }

   LaserScan(const LaserScan& scan)
      : max_distance_(scan.max_distance_)
   {
      for(int i = 0; i < scan.size(); i++)
            points_.push_back(scan.points_[i]);
   }

   LaserScan& operator=(const LaserScan& scan)
   {
      points_.clear();
      for(int i = 0; i < scan.size(); i++)
            points_.push_back(scan.points_[i]);
      return *this;
   }

   LaserScan subtractLaserScan(const LaserScan& scan, double distance_threshold = 0.1)
   {
      LaserScan temp(max_distance_, points_.size());
      if(scan.size() == points_.size())
      {
         for(int i = 0; i < points_.size(); i++)
         {
            if(scan[i].getAngle() == points_[i].getAngle())
            {
               double distance = std::fabs(scan[i].getDistance() - points_[i].getDistance());
               if(distance > distance_threshold)
                  temp.push_back(points_[i]); 
            }
         }
      }
      return temp;
   }

   double regionDistance(const double &start, const double &stop) const
   {
      int limit1 = 0, limit2 = 0;
      if (!this->points_.empty())
      {
         for (int i = 0; i < points_.size(); i++)
         {
            if (points_[i].getAngle() < start)
               limit1++;
            if (points_[i].getAngle() < stop)
               limit2++;
         }

         if (limit1 > limit2)
            return getMinDistance(limit2, limit1);

         return getMinDistance(limit1, limit2);
      }
      return max_distance_;
   }

   double regionAverageDistance(const double &start, const double &stop) const
   {
      int limit1 = 0, limit2 = 0;
      if (!this->points_.empty())
      {
         for (int i = 0; i < points_.size(); i++)
         {
            if (points_[i].getAngle() < start)
               limit1++;
            if (points_[i].getAngle() < stop)
               limit2++;
         }

         if (limit1 > limit2)
            return getAverageDistance(limit2, limit1);

         return getAverageDistance(limit1, limit2);
      }
      
      return max_distance_;
   }

   void updateLaserscan(const sensor_msgs::LaserScan::ConstPtr &msg)
   {
      points_.clear();

      for(int i = 0; i < msg->ranges.size(); i++)
      {
         if(msg->ranges[i] > 0.0)
         {
            double angle = msg->angle_min + msg->angle_increment * (double)i;
            double distance = msg->ranges[i];
            LaserScanPoint point(distance, angle);
            points_.push_back(point);
         }
      }
   }

   void clear() { points_.clear(); }

   void push_back(const LaserScanPoint& point)
   {
      points_.push_back(point);
   }

   const LaserScanPoint& operator[](unsigned int index) const
   {
      return points_[index];
   }

   LaserScanPoint& operator[](unsigned int index)
   {
      return points_[index];
   }

   unsigned int size() const { return points_.size(); }

   bool empty() const { return points_.empty(); }

   ~LaserScan()
   {
      points_.clear();
   }

   private:
   
   double getMinDistance(int start_pos, int end_pos) const
   {
      double distance = max_distance_;
      for(int i = start_pos;  i < end_pos; i++)
      {
         if(points_[i].getDistance() < distance)
            distance = points_[i].getDistance();
      } 
      return distance;
   }

   double getAverageDistance(int start_pos, int end_pos) const
   {
      double distance_sum = 0.0;
      int n = 0;
      for(n = start_pos;  n < end_pos; n++)
      {
         distance_sum += points_[n].getDistance();
      } 
      if(n != 0)
         return distance_sum / (double)n;
      return max_distance_;
   }
};

} // namespace laserscan
#endif