#ifndef LASER_SCAN_H
#define LASER_SCAN_H

#include <string>
#include <vector>
#include "ros/ros.h"


namespace laserscan
{

class LaserScan
{
private:
   std::vector<double> distances_;
   std::vector<double> angles_;

public:
   LaserScan(){}

   float regionDistance(const float &start, const float &stop)
   {
      int i = 0, j = 0;
      if (!this->angles_.empty())
      {
         for (auto &angle : this->angles_)
         {
            if (angle < start)
               i++;
            if (angle < stop)
               j++;
         }

         if (i > j)
            return *std::min_element(this->distances_.begin() + j, this->distances_.begin() + i);

         return *std::min_element(this->distances_.begin() + i, this->distances_.begin() + j);
      }
      return 8.0;
   }

   void updateLaserscan(const sensor_msgs::LaserScan::ConstPtr &msg)
   {
      this->distances_.clear();
      this->angles_.clear();

      for (int i = 0; i < msg->ranges.size(); i++)
      {
         this->distances_.push_back(msg->ranges[i]);
      }

      float angle = msg->angle_min;
      for (; angle < msg->angle_max; angle += msg->angle_increment)
      {
         this->angles_.push_back(angle);
      }
   }

   ~LaserScan()
   {
      distances_.clear();
      angles_.clear();
   }

private:
   
};

} // namespace laserscan
#endif