# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

[//]: # (Image References)

[image1]: ./images/TargetMiles.png "TargetMile"
[image3]: ./images/Image3.png "Image3"


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

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

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

### Model Documentation:
For building this project i followed the project walkthrough provided in the end of the project lesson.

In this project i used the freent co-ordinates to simplifies the path planing. The total track is approxiomately 6945 meters long and this is assigned to max_s variable.
1. Staying in a line:

To stay in a line i used the d value which shows the distance away from center divider. Here each line is 4 meters wide and total lines are 3 (Left, Right and center). so the value of d for left line is 0 < d < 4, center is 4 < d < 8 and right 8 < d < 12. Co-ordinates of (s,d) will specifies the position of car.

2. Going through the full track: 
Using frent co-ordinate system generete five points to pass through the track. Using the spline library, construct points from the 29m of the spline path. here i have taken the target lane spaced as 29m.
Add the points on spline to next_x_vals and next_y_vals. spline was used to generate the trajectory. Spline used mainly for starting reference trajectory  and waypoints (x,y). 

3. Velocity under the speed limit:n
Used the  variable double ref_vel to set the speed limit under the 50. i started by assigning the 0 to this variable and slowly increased to 49.5 which is slightly lower than target speedlimit.
When adding the path points to the path, i used the formula (target_dist/(0.02*ref_vel/2.24)), Here division by 2.24 converts the ref_vel from miles to meters per second.

4. Avoid the collisions with the cars in our car line:
To avoid the collisions with other cars i followed the steps below
- check if our car is too close with the other cars in the same line
- if the car is too_close within the same line in front cars then there is a chance of collision, in this case our car should slow down. This imeplemeted in below code
if (too_close) {
                ref_vel -= .324;
            } else if(ref_vel < 49.5) {
                ref_vel += 0.324;
            }
Use the data in the sensor_fusion vector to predict where each other car will be in the next timestep. By using the Frenet coordinates:
Use other car's s to determine whether it is close to our car.
Use other car's d to determine which lane the other car is in.

5. Change lane lines:
In general the car will saty in the same line if there is no traffic. If our car lane will change when there traffi ahead even when our car speed decreased. It will try to find a lane can safely move into
- Drive in same lane and drive with velocity speed  as long as possible.
- If there is a traffic ahead, flag for lane change . I used the method checkLaneChangePossibility to check the lane possibilty and do the lane shift.

Conclusion:
Finally the car able to drive successfully 4.5 miles without any collisisons. During the traffic in the same lane, our car is able to change the lane smoothly. It is required some improvmenets to driev entire track without any collisions. 

Test Images:

[//]: # (Image References)

![alt text][image1]"
![alt text][image3]"