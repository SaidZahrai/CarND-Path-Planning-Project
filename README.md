This is a project in Udacity Nanodegree for Self-driving car, forked from [CarND-Path-Planning-Projec](https://github.com/udacity/CarND-Path-Planning-Project).


# Path planning
The goal of this project is to implement a path planner that with help of sensor inputs can find a collision free way forward in a highway in a trafic simulator. This has been achieved in the following way:

1. A class vehicle, has been defined that wraps functions related to the vehicle [./src/vehicle.cpp](./src/vehicle.cpp). This class, in addition to collecting data necessary for control of the vehicle, implements four important functions:

    a. update(...) is to be called every step to update the data with the fresh ones.

    b. cruise_control() drives the vehicle at a constant speed, set to 95% of maximum allowed speed, i.e. 50 MPH, if there are no obstacles on the path and otherwise, adjusts the speed to the front car.

    c. successor_states() implements the state machine and for each state of the vehicle returns next possible stated.

    d. set_next_state() goes through all possible states, calculates the cost for each of them and selects the one with lowest cost as next step.

2. [./src/cost.cpp](./src/cost.cpp) implements the cost functions a function that calls all of them and calculates the total cost. Four different costs are considered:

    a. lane_cost() sets the cost to 0 for lane 1 and to 1 otherwise, meaning the middle lane is the prefered one.

    b. right_passage_cost() makes it more favorable to pass the car in the front from the left side by adding a cost contribution if there is a car to be passed from right.

    c. inefficiency_cost() penalizes a driving at a speed lower that the maximum allowed speed. A fourth degree polynomial is used to push the system to choose a path with higher speed.

    d. collision_cost() eliminates risks of collision through pnalizing the states where the car is driving to places that are occupied by other costs.

    Finally, these costs are weighted by 0.5, 4.0, 0.25, 10.0, respectively.

3. src/main.cpp was first completed following the instructions given in the video and then modified as follows:

    a. According to the instructions, the path planner was adding points that were consumed between two calls. This made the reaction slower that could be adjusted for by less changes and less aggressive driving. There was also an impact on the braking distance. Path points were modified only after that they were consumed. As a simple adjustment, I decided to replan the path by keeping two points only. Line 39 [./src/main.cpp](./src/main.cpp).

    b. The sensor data is used to identify the closest cars in each lane. The information is collected and stored in two vectors that is sent to the vehicle object by the function update. Line 40 [./src/main.cpp].

    c. The speed reduction and acceleration is made by the cruise_control funtion of the cehicle object. Lines 111 [./src/main.cpp](./src/main.cpp).

    d. I used a different method for path discretization. I have both x and y as function of s and then, I choose delta values of s by multiplying the speed and the time difference between two points (0.02 seconds). The spline function can then be used to find corresponding x and y values. Line 111 [./src/main.cpp](./src/main.cpp).

4. CMakeLists.txt is modified so that Vehicle class and cost functions can be compiled and linked with the other object files.

# Build, compilation and installation on Ubuntu
After cloning the repository, create a new directory with the name build, change the directory to build and run `cmake` as shown below:

`mkdir build`

`cd build`

`cmake ..`

This will create a make file that can be used to compile and link the program using `make`. For that, while being in build directory, simply run:

`make`

Running `make` will compile the c++ files and try to link them. Assuming that all dependencies described in [Udacity's project repository](https://github.com/udacity/CarND-Path-Planning-Project) are satisfied, and especially [micro WebSockets](https://github.com/uNetworking/uWebSockets) is installed, make will produce an executable file called `path_planning`. If you do not have [micro WebSockets](https://github.com/uNetworking/uWebSockets) installed, please follow [Udacity's instructions](https://classroom.udacity.com/nanodegrees/nd013/parts/168c60f1-cc92-450a-a91b-e427c326e6a7/modules/1c4c9856-381c-41b8-bc40-2c2fd729d84e/lessons/3feb3671-6252-4c25-adf0-e963af4d9d4a/concepts/7dedf53a-324a-4998-aaf4-e30a3f2cef1d) to have it properly installed.

# Running the planner
Running the controller requires connection with [Udacity's simulator](https://github.com/udacity/self-driving-car-sim/releases). I had my simulator on Windows and the controller on Ubuntu running on [VirtualBox](https://www.virtualbox.org/wiki/Downloads).

The execution can start by executing `./path_plnning` from the build directory, which runs the planner: 
<p align="center">
<img src="./images/normal_start.png" width="600"/>
</p>

<p align="center">
<img src="./images/pid_controlled.png" width="600"/>
</p>

# Reflections


# Results

