#ifndef State_H
#define State_H
#include <math.h>
#include "lane.h"
#include "localization_data.h"

class State {
  // Abstract class for finite state machine states
  public:      
    virtual double TargetD() = 0;
    virtual double TargetSpeed() = 0;
    virtual double ProjectedS() = 0;
  protected:
    State(LocalizationData localization_data) : localization_data_(localization_data) {}
    LocalizationData localization_data_;
    const double kTolerableAcceleration_ = 0.224;
    const double kLaneWidth_ = 4;
    const double kSecondsPerFrame_ = 0.02;
};

#endif /* State_H */