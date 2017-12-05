#ifndef FrenetCartesianConverter_H
#define FrenetCartesianConverter_H
#include <vector>

class FrenetCartesianConverter {
public:
  FrenetCartesianConverter(std::vector<double> &map_waypoints_s, std::vector<double> &map_waypoints_x, std::vector<double> &map_waypoints_y);

  virtual ~FrenetCartesianConverter();

  std::vector<double> FrenetToCartesian(double s, double d);

private:
  std::vector<double> map_waypoints_s_;
  std::vector<double> map_waypoints_x_;
  std::vector<double> map_waypoints_y_;
};

#endif /* FrenetCartesianConverter_H */
