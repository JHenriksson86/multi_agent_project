#ifndef OBJECT_TRACKING_H
#define OBJECT_TRACKING_H

#include <list>
#include <cmath>
#include <string>
#include <iostream>

#include "ros/ros.h"
#include "eigen3/Eigen/Dense"

#include "navigation.h"
#include "laser_scan.h"

namespace robot
{
   class TrackedObject
   {
      private:
      Eigen::Vector2d coordinates_;
      double log_odds_;

      public:
      TrackedObject(Eigen::Vector2d coordinates,  double probability = 0.60)
      {
         this->coordinates_ = coordinates;
         this->log_odds_ = toLogOdds(probability);
      }

      TrackedObject(double x, double y, double probability = 0.60)
      {
         this->coordinates_ = Eigen::Vector2d(x,y);
         this->log_odds_ = toLogOdds(probability);
      }

      double getProbability() const { return toProbability(log_odds_); }

      void detected() 
      {
         log_odds_ += toLogOdds(0.60);
      }

      void notDetected()
      {
         log_odds_ += toLogOdds(0.10);
      }

      Eigen::Vector2d getCoordinates() const { return coordinates_; }

      std::string to_string()
      {
         std::string result = "x=" + std::to_string(coordinates_[0]) + 
         " y=" + std::to_string(coordinates_[1]) + " prob=" + std::to_string(toProbability(log_odds_));
         return result;
      }

      private:

      double toLogOdds(double probability) const
      {
         return std::log(probability/(1.0-probability));
      }

      double toProbability(double log_odds) const
      {
         double odds = std::exp(log_odds);
         return odds/(odds + 1.0);
      }
   };

   class ObjectTracking
   {
      private:
      Navigation*                navigation_;
      std::list<Eigen::Vector2d> auctioned_objects_;
      std::list<TrackedObject>   tracked_objects_;
      Eigen::Vector2d            cluster_point_;
      double                     distance_limit_;
      double                     probability_limit_;
      bool                       cluster_point_active_;

      public:

      ObjectTracking(Navigation* navigation, double distance_limit = 1.0, double probability_limit = 0.99)
      {
         this->navigation_ = navigation;
         this->distance_limit_ = distance_limit;
         this->probability_limit_ = probability_limit;
         this->cluster_point_ = Eigen::Vector2d::Zero();
         this->cluster_point_active_ = false;
      }

      void setClusterPoint(const Eigen::Vector2d& object)
      {
         ROS_DEBUG("ObjectTracking::setClusterPoint()");
         this->cluster_point_ = object;
         this->cluster_point_active_ = true;
      }

      void disableClusterPoint()
      {
         ROS_DEBUG("ObjectTracking::disableClusterPoint()");
         this->cluster_point_ = Eigen::Vector2d::Zero();
         this->cluster_point_active_ = false;
      }

      void addAuctionedObject(const Eigen::Vector2d& object)
      {
         ROS_DEBUG("ObjectTracking::addAuctionedObject()");
         std::list<TrackedObject>::iterator object_it = 
                  findObject(tracked_objects_, object);
         if(object_it != tracked_objects_.end())
         {
            auctioned_objects_.push_back(object);
            tracked_objects_.erase(object_it);
         }
         else
         {
            ROS_WARN("ObjectTracking::addAuctionedObject() object couldn't be found");
         }
      }

      bool removeAuctionedObject(const Eigen::Vector2d& object)
      {
         ROS_DEBUG("ObjectTracking::removeAuctionedObject()");
         std::list<Eigen::Vector2d>::iterator it = findObject(auctioned_objects_, object);
         if(it != auctioned_objects_.end())
         {
            auctioned_objects_.erase(it);
            return true;
         }
         return false;
      }

      void update(const LaserScan& object_candidates)
      {
         ROS_DEBUG("ObjectTracking::update() number of candidates %d.", object_candidates.size());
         std::list<Eigen::Vector2d> detected_objects;
         filterCandidates(object_candidates, &detected_objects);
         
         if(!tracked_objects_.empty())
         {
            std::list<TrackedObject>::iterator tracked_iterator = tracked_objects_.begin();
            while(tracked_iterator != tracked_objects_.end())
            {
               std::list<Eigen::Vector2d>::iterator match_it = 
                  findObject(detected_objects, tracked_iterator->getCoordinates());
               if(match_it != detected_objects.end())
               {
                  ROS_DEBUG("ObjectTracking::update() match found.");
                  tracked_iterator->detected();
                  detected_objects.erase(match_it);
                  tracked_iterator++;
               }
               else
               {
                  ROS_DEBUG("ObjectTracking::update() match not found.");
                  tracked_iterator->notDetected();
                  if(tracked_iterator->getProbability() < 0.2)
                  {
                     ROS_DEBUG("ObjectTracking::update() probability to low erasing object.");
                     std::list<TrackedObject>::iterator erase_it = tracked_iterator;
                     tracked_iterator++;
                     tracked_objects_.erase(erase_it);
                  }
                  else
                  {
                     tracked_iterator++;
                  }
               }
            }
         }

         if(!detected_objects.empty())
         {
            for(auto it = detected_objects.begin(); it != detected_objects.end(); it++)
            {
               tracked_objects_.push_back(TrackedObject(*it));
            }
         }
      }

      bool foundObject()
      {
         ROS_DEBUG("ObjectTracking::foundObject()");
         std::list<TrackedObject>::iterator listIt = tracked_objects_.begin();
         for(; listIt != tracked_objects_.end(); listIt++)
         {
            if(listIt->getProbability() > probability_limit_)
            {
               ROS_DEBUG("ObjectTracking::foundObject() object with %.2f probability found.", listIt->getProbability());
               return true;
            }
         }
         return false;
      }

      Eigen::Vector2d getBestObject()
      {
         ROS_DEBUG("ObjectTracking::getBestObject()");
         std::list<TrackedObject>::iterator bestIt = tracked_objects_.end();
         double best_distance = -1.0;
         std::list<TrackedObject>::iterator listIt = tracked_objects_.begin();
         for(; listIt != tracked_objects_.end(); listIt++)
         {
            if(listIt->getProbability() > probability_limit_)
            {
               double distance = navigation_->getDistanceToCoordinate(listIt->getCoordinates());
               if(bestIt != tracked_objects_.end())
               {
                  if(distance < best_distance)
                  {
                     best_distance = distance;
                     bestIt = listIt;
                  }
               }
               else
               {
                  best_distance = distance;
                  bestIt = listIt;  
               }
            }
         }
         return bestIt->getCoordinates();
      }
      
      void printObjects()
      {
         std::cout << "Tracked objects:\n";
         for(auto it = tracked_objects_.begin(); it != tracked_objects_.end(); it++)
         {
            std::cout << "\t" << it->to_string() << "\n";
         }
         std::cout << "Auction objects list size=" << std::to_string(auctioned_objects_.size()) << "\n";
         std::cout << "End\n";
      }

      private:

      void filterCandidates(
         const LaserScan& object_candidates, 
         std::list<Eigen::Vector2d>* detected_objects
      )
      {
         ROS_DEBUG("ObjectTracking::filterCandidates() entering.");
         for(int i = 0; i < object_candidates.size(); i++)
         {
            LaserScanPoint candidate(object_candidates[i]);
            int close_objects = 0;
            for(int j = 0; j < object_candidates.size(); j++)
            {
               if(i != j)
               {
                  if(candidate.getDistance(object_candidates[j]) < 0.2)
                     close_objects++;
               }
            }
            if(close_objects > 2)
            {
               Eigen::Vector2d new_object_coordinates = getObjectWorldCoordinates(candidate.getCartesianCoordinates());
               if(
                  candidate.getDistance() < distance_limit_ 
                  && !isOnList(*detected_objects, new_object_coordinates)
                  && !globalExceptions(new_object_coordinates)
               )
               {
                  detected_objects->push_back(new_object_coordinates);
               }
            }
         }
         ROS_DEBUG("ObjectTracking::filterCandidates() leaving.");
      }

      bool globalExceptions(const Eigen::Vector2d& object)
      {
         if(!isOnList(auctioned_objects_, object))
         {
            if(cluster_point_active_ && getObjectDistance(object, cluster_point_) < 0.2)
            {
               return true;
            }  
            else
            {
               return false;
            }
         }
         else
         {
            return true;
         }
      }

      std::list<Eigen::Vector2d>::iterator findObject(
         std::list<Eigen::Vector2d>& object_list, 
         const Eigen::Vector2d& object
      )
      {
         ROS_DEBUG("ObjectTracking::findObject()");
         std::list<Eigen::Vector2d>::iterator listIt = object_list.begin();
         for(; listIt != object_list.end(); listIt++)
         {
            if(getObjectDistance(object, *listIt) < 0.1)
            {
               return listIt;
            }
         }
         return listIt;
      }

      std::list<TrackedObject>::iterator findObject(
         std::list<TrackedObject>& object_list, 
         const Eigen::Vector2d& object
      )
      {
         ROS_DEBUG("ObjectTracking::findObject()");
         std::list<TrackedObject>::iterator listIt = object_list.begin();
         for(; listIt != object_list.end(); listIt++)
         {
            if(getObjectDistance(object, listIt->getCoordinates()) < 0.1)
            {
               return listIt;
            }
         }
         return listIt;
      }

      bool isOnList(std::list<Eigen::Vector2d>& object_list, const Eigen::Vector2d& object)
      {
         ROS_DEBUG("ObjectTracking::isOnList()");
         std::list<Eigen::Vector2d>::iterator it = findObject(object_list, object);
         if(it != object_list.end())
            return true;
         return false;
      }

      bool isOnList(std::list<TrackedObject>& object_list, const Eigen::Vector2d& object)
      {
         ROS_DEBUG("ObjectTracking::isOnList()");
         std::list<TrackedObject>::iterator it = findObject(object_list, object);
         if(it != object_list.end())
            return true;
         return false;
      }

      double getObjectDistance(const Eigen::Vector2d& object1, const Eigen::Vector2d& object2)
      {
         ROS_DEBUG("ObjectTracking::getObjectDistance()");
         return (object1 - object2).squaredNorm();
      }

      Eigen::Vector2d getObjectWorldCoordinates(const Eigen::Vector2d& object) 
      {
         ROS_DEBUG("ObjectTracking::getObjectWorldCoordinates()");
         return navigation_->convertToWorldCoordinate(object);
      }

   };
}; // namespace robot

#endif