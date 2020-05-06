#ifndef FUZZY_H
#define FUZZY_H

#include <cmath>

namespace robot
{

struct FuzzyFunctions
{
   static double OR(double x, double y)
   {
      return std::fmax(x, y);
   }

   static double AND(double x, double y)
   {
      return std::fmin(x, y);
   }

   static double OR3(double x, double y, double z)
   {
      return OR(x, OR(y, z));
   }

   static double AND3(double x, double y, double z)
   {
      return AND(x, AND(y, z));
   }

   static double NOT(double x)
   {
      return 1.0 - x;
   }

   static double rampUp(double x, double start, double stop)
   {
      if (x >= start)
         return 0.0;
      else if (x < stop)
         return 1.0;
      else
         return ((x - start) / (stop - start));
   }

   static double rampDown(double x, double start, double stop)
   {
      if (x >= start)
         return 1.0;
      else if (x < stop)
         return 0.0;
      else
         return ((x - stop) / (start - stop));
   }
};

} // namespace robot

#endif