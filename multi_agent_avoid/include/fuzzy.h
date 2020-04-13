#ifndef FUZZY_H
#define FUZZY_H

#include <cmath>

namespace fuzzy {
  class FuzzyFunctions {

    public:

    #define CALL_MEMBER_FN(object,ptrToMember)  ((object).*(ptrToMember))

    typedef double (FuzzyFunctions::*rampFunctionPointer) (double, double, double);

    enum rampFunction{
      standard,
      sigmoid
    };
    
    FuzzyFunctions(rampFunction function = standard, double sigmoidK = 5.0){
      this->sigmoidK_ = sigmoidK;
      if(function == standard){
        rampUpPtr_ = &FuzzyFunctions::standardRampUp;
        rampDownPtr_ = &FuzzyFunctions::standardRampDown;
      }
      else{
        rampUpPtr_ = &FuzzyFunctions::sigmoidRampUp;
        rampDownPtr_ = &FuzzyFunctions::sigmoidRampDown;
      }
    }

    static double OR(double x, double y){
      return std::fmax(x, y);
    }

    static double AND(double x, double y){
      return std::fmin(x, y);
    }

    static double NOT(double x){
      return 1.0 - x;
    }

    double rampUp(double x, double start, double stop){
      return CALL_MEMBER_FN(*this, rampUpPtr_)(x, start, stop);
    }

    double rampDown(double x, double start, double stop){
      return CALL_MEMBER_FN(*this, rampDownPtr_)(x, start, stop);
    }

    private:
    
    rampFunctionPointer rampUpPtr_;
    rampFunctionPointer rampDownPtr_;
    double sigmoidK_;

    double standardRampUp(double x, double start, double stop){
      if(x >= start) return 0.0;
      else if(x < stop) return 1.0;
      else return ((x - start) / (stop - start));
    }
    
    double standardRampDown(double x, double start, double stop){
      if(x >= start) return 1.0;
      else if(x < stop) return 0.0;
      else return ((x - stop) / (start - stop));
    }

    double sigmoidRampUp(double x, double start, double stop){
      //Growth rate
      const double k = sigmoidK_ / std::sqrt(std::pow(start, 2) + std::pow(stop, 2));
    
      //Midpoint
      const double x0 = std::fabs(stop + start) / 2.0;

      return 1.0 / (1.0 + std::pow(M_E, k * (x - x0)));
    }

    double sigmoidRampDown(double x, double start, double stop){
      //Growth rate
      const double k = sigmoidK_ / std::sqrt(std::pow(start, 2) + std::pow(stop, 2));
      
      //Midpoint
      const double x0 = std::fabs(stop + start) / 2.0;

      return 1.0 / (1.0 + std::pow(M_E, -k * (x - x0)));
    }

  };
}

#endif