#ifndef LaneChangeRight_H
#define LaneChangeRight_H
#include "state.h"

class LaneChangeRight: public State {
   public: 
      LaneChangeRight(LocalizationData localization_data) : State(localization_data) {}
      ~LaneChangeRight() {}     

      double TargetD() { 
        return localization_data_.d + kLaneWidth_;
      }

      double TargetSpeed() {
        return std::min(localization_data_.speed + kTolerableAcceleration_, localization_data_.speed_limit);
      }
      
      double ProjectedS(){
        return localization_data_.s + TargetSpeed() * kSecondsPerFrame_;
      }
};

#endif /* LaneChangeRight_H */