#ifndef LaneKeep_H
#define LaneKeep_H
#include "state.h"

class LaneKeep : public State {
   public:    
      LaneKeep(LocalizationData localization_data) : State(localization_data) {}
      ~LaneKeep() {}
      
      double TargetD() { 
        return localization_data_.d;
      }

      double TargetSpeed() {
        return std::min(localization_data_.speed + kTolerableAcceleration_, localization_data_.speed_limit);
      }
      
      double ProjectedS(){
        return localization_data_.s + TargetSpeed() * kSecondsPerFrame_;
      }
};

#endif /* LaneKeep_H */