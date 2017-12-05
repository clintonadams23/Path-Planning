#include "frenet_cartesian_converter.h"
#include <math.h>
using std::vector;

FrenetCartesianConverter::FrenetCartesianConverter(vector<double>& map_waypoints_s, vector<double>& map_waypoints_x, vector<double>& map_waypoints_y) {
	map_waypoints_s_ = map_waypoints_s;
	map_waypoints_x_ = map_waypoints_x;
	map_waypoints_y_ = map_waypoints_y;
}

FrenetCartesianConverter::~FrenetCartesianConverter() {}

vector<double> FrenetCartesianConverter::FrenetToCartesian(double s, double d) {
	int prev_wp = -1;

	while(s > map_waypoints_s_[prev_wp+1] && (prev_wp < (int)(map_waypoints_s_.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%map_waypoints_x_.size();

	double heading = atan2((map_waypoints_y_[wp2]-map_waypoints_y_[prev_wp]),(map_waypoints_x_[wp2]-map_waypoints_x_[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-map_waypoints_s_[prev_wp]);

	double seg_x = map_waypoints_x_[prev_wp]+seg_s*cos(heading);
	double seg_y = map_waypoints_y_[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-M_PI/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};
}