#ifndef LaneChangeLeft_H
#define LaneChangeLeft_H
#include "state.h"

class LaneChangeLeft : public State {
   public:
      LaneChangeLeft(LocalizationData localization_data) : State(localization_data){}
      ~LaneChangeLeft() {} 

      double TargetD() { 
        return localization_data_.d - kLaneWidth_;
      }

      double TargetSpeed() {
        return std::min(localization_data_.speed + kTolerableAcceleration_, localization_data_.speed_limit);
      }

      double ProjectedS(){
        return localization_data_.s + TargetSpeed() * kSecondsPerFrame_;
      }
};

#endif /* LaneChangeLeft_H */