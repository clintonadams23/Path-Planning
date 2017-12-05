#ifndef FiniteStateFactory_H
#define FiniteStateFactory_H
#include <memory>
#include <vector>
using std::vector;
#include "state.h"
#include "localization_data.h"

class FiniteStateFactory {
  public:      
    static std::unique_ptr<State> GetBestState(LocalizationData& localization_data, vector<vector<double>>& sensor_fusion);
  private:
    FiniteStateFactory() {};
    static double CalculateCost(State* state, LocalizationData& localization_data, vector<vector<double>>& sensor_fusion);
    static double OtherCarInLaneCost(State* state, vector<vector<double>>& sensor_fusion, double weight);
    static double OffroadCost(State* state, double weight);
    static double SpeedCost(State* state, LocalizationData& localization_data, double weight);
    static double LaneChangeCost(State* state, LocalizationData& localization_data, double weight);
};

#endif /* FiniteStateFactory_H */