## 6. Car Path Planning

### Self-Driving Car Engineer Nanodegree Program - Term 3

### Goals

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### Demo

The final program achieves successful self-driving around the tracks with a reference max speed of 50 mph, without accident for over 1 hour and traveling over 44 miles:

[//]: # (Image References)
[image1]: ./images/car-path-planning-2018-11-12t12-50-37.png "example driving result"

![alt text][image1] <br/>

An example Youtube vedio showing the car self driving behaviors is at:

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/lllzrflILoU/0.jpg)](https://www.youtube.com/watch?v=lllzrflILoU)

### Path Planning

There have been 5 attempts to achieve a workable path planning solution. Please see the respective comments in source code main.cpp for the implementation details.

#### Attempt 1: Make the car move from 0 speed

Feed a constant speed of 49.5 mph and a straight path ahead, though the car ends up in the woods. A sudden 0 to 49.5 mph speed increase also causes big acceleration jerks.

#### Attempt 2: Make the car stay in the middle lane 

Feed a constant speed of 49.5 mph and a frenet coordinate d of 6 to make the car stay driving in the middle lane. The car follows the lane well, though it ends up rear crash the front car due to the contant speed of 49.5 mph. The 0 to 49.5 mph acceleration jerk problem remains.

#### Attempt 3: Use spline to smooth car moving path

Fit the way points of the tracks with respective to the current car position, and make a smoother path for car to move. This is also to prepare a smoother lane change for the car later.

#### Attempt 4: Catch up or slow down based on the distance to the front car

Stay in the middle lane, calculate the distance to the car right in the front with the fusion data. If not too close and the car is within the max speed limit, catch up speed. Slow down if the car is too close. The car now maintains the lane without acceleration jerks.

#### Attempt 5: Change lane if the front car causes slow down

Calculate the distance to the car right in the front with the fusion data, and detect if it is too close and will cause a slow down or a lane change.

Calulate the distances to the cars in the three lanes traveling in the same direction, with the fushion data, and detect if any of the lane has cars too close. 

If no car on the left is too close, and the current car speed is over 45.0 mph, the car will make a lane change to the left. Otherwise, if no car on the right is too close, the the current car speed is over 45.0 mph, the car will make a lane change to the right. 

The car will not change lane if it has just changed a lane 1000 message beats (20 seconds) ago, to avoid behaviors of aggressive lane changes.

#### Reflections: Behavioral path planning with path generation and machine learning 

The attempt 5 essentially encodes a rule-based path planning algorithm. It works well for the simulated environment. The real world driving hower is much more complex. For example a car crash in the middle lane may need us to change lane quickly to the right most lane, without considering anything if the car has changed the lane a few seconds ago. The path planning can be made more robust if more previous data have been collected of the actual road conditions, along with behaviours of other drivers, and importantly what the right responses the car should react. A sophisticated machine learning model can be built, for example, to decide if the car should change the lane to the left or to the right, when the car is in the middle lane. 

Lane change is essentailly a decision on the best path among a few possible pathes that the car can possibly head to. To generate path predictions, we extract features like the position, speed, and yaw angles of the cars in the next few lanes close by based on the fusion data. We use spline to extrapolate the trajectories up to a specific horizon (a few seconds), the behaviour planner defines a set of candidate targets: lane, speed, and time for the driving maneuvers:

- for every candidate path a trajectory is fitted,
- for every candidate trajectory a cost is computed,
- the best trajector with the lowest cost is selected

The design of the algorithm is largely in the extraction of the right features and the cost function. A machine learning or even simple logistic regression algorithm can be helpful in the path calculatins and predictions.

### Simulator
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

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

### Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

### Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

### Dependencies

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

### Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

### Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

### Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


### Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

### How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

