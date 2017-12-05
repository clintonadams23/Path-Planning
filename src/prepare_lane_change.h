#ifndef PrepareLaneChange_H
#define PrepareLaneChange_H
#include "state.h"

class PrepareLaneChange: public State {
   public:
      PrepareLaneChange(LocalizationData localization_data) : State(localization_data) {}
      ~PrepareLaneChange() {}

      double TargetD() { 
        return localization_data_.d;
      }

      double TargetSpeed() {
        return localization_data_.speed - kTolerableAcceleration_;
      }

      double ProjectedS(){
        return localization_data_.s + TargetSpeed() * kSecondsPerFrame_;
      }
};

#endif /* PrepareLaneChange_H */