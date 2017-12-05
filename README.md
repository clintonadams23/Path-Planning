# Path-Planning
Use sensor fusion, localization, and prediction to safely drive on a highway

## Goals
* Safely driving on the highway in the simulator is defined as meeting the following goals:

* The car doesn't drive faster than the speed limit. Also the car isn't driving much slower than speed limit unless obstructed by traffic.

* The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.

* The car must not come into contact with any of the other cars on the road.

* The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road.

* The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic.

## Model Documentation
The vehicle uses a spline to interpolate between consecutive map waypoints and provide a smooth trajectory. The previous path is also used to further smooth out the trajectory and compensate for latency.

I used a finite state machine approach with the information provided by the simulator to generate an optimal trajectory. The car has four possible states it can be in:
#### Lane Keep
In this state the cars intended lane will remain the same and it will accelerate until it reaches the speed limit.

#### Lane Change Left
In this state the cars intended lane will be the current frenet d value minus the width of a lane. It will accelerate until it reaches the speed limit.

#### Lane Change Right
In this state the cars intended lane will be the current frenet d value plus the width of a lane. It will accelerate until it reaches the speed limit.

#### Prepare Lane change
In this state the cars intended lane will remain the same. It will deccelerate as long as it remains in this state.

Cost functions are continually evaluated on each the possible states and the state with the lowest current total cost is selected. The costs functions are based on the following:

#### Offroad Cost
If the state's frenet d value is outside of the bounds of the highway, a high cost is applied. Otherwise no cost is applied.

#### Lane Change Cost
A small cost is applied to a lane change to limit excessive lane switching.

#### Other Car In Lane Cost
If there is another vehicle in the state's destination lane, a cost is applied that is inversely proportional to the square of that car's distance. This makes the cost of collisions prohibitively high and also encourages early lane changing to the lane with the car furtherest ahead. 

#### Speed Cost
This cost is proportional to the square of the difference between the state's speed and the speed limit. This encourages the car to try lane changing if possible before slowing down.

## Inputs
#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Other considerations

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. 

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points used so that the car has a smooth transition. 


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.
