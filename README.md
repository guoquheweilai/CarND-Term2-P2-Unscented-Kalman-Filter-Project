# CarND-Term2-P1-Extended-Kalman-Filter  
## Overview  
In this project, you are going to implement the unscented Kalman filter in C++. You will be tracking the bicycle's position and velocity and predicting the positon of the object.  
This project is based on the same structure as the [extended Kalman filter](https://github.com/ikcGitHub/CarND-Term2-P1-Extended-Kalman-Filter). It uses a main file that calls a function called ProcessMeasurement. Anything important happens in this function. The function is part of the class ukf.  
There is a simulator provided by Udacity ([Term 2 Simulator Release](https://github.com/udacity/self-driving-car-sim/releases/)) which can generate noisy LIDAR and RADAR measurements. And you will be using those measurements to predict your object position.  
Here is the link to the [orginal repository](https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project) provided by Udaciy.  
## Prerequisites/Dependencies  
* cmake: 3.5  
  * All OSes: [click here for installation instructions](https://cmake.org/install/)  
* make: 4.1 (Linux and Mac), 3.81 (Windows)  
  * Linux: make is installed by default on most Linux distros  
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)  
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)  
* gcc/g++: 5.4  
  * Linux: gcc / g++ is installed by default on most Linux distros  
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features)  
  * Windows: recommend using [MinGW](http://www.mingw.org/)  
## Setup Instructions (abbreviated)  
1. Meet the `Prerequisites/Dependencies`  
2. Intall `uWebSocketIO ` on your system  
  2.1 Windows Installation  
  2.1.1 Use latest version of Ubuntu Bash 16.04 on Windows 10, here is the [step-by-step guide](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) for setting up the utility.  
  2.1.2 (Optional) Check your version of Ubuntu Bash [here](https://www.howtogeek.com/278152/how-to-update-the-windows-bash-shell/).  
3. Open Ubuntu Bash and clone the project repository  
4. On the command line execute `./install-ubuntu.sh`  
5. Build and run your code.  
## Project Description  
- [main.cpp](./src/main.cpp):Reads in data, calls a function to run the Unscented Kalman filter, calls a function to calculate RMSE
- [ukf.cpp](./src/ukf.cpp): Initializes the Unscented Kalman filter, calls the predict and update function, defines the predict and update functions
- [tools.cpp](./src/tools.cpp): Function to calculate RMSE
- [README.md](./README.md): Writeup for this project, including setup, running instructions and project rubric addressing.  
- [CMakeLists.txt](./CMakeLists.txt): `CMakeLists.txt` file that will be used when compiling your code (you do not need to change this file)
## Run the project  
* Clone this respository
* At the top level of the project repository, create a build directory: `mkdir build && cd build`
* In `/build` directory, compile yoru code with `cmake .. && make`
* Launch the simulator from Windows
* Execute the run command for the project `./UnscentedKF` (Make sure you also run the simulator on the Windows host machine) If you see * * this message, it is working `Listening to port 4567 Connected!!!`
## PROJECT PASSING CRITERIA  
There are several criteria that must be fulfilled to pass the project.
- The overall processing chain (prediction, laser update or radar update depending on measurement type) must be correct.
- The student is not allowed to use values from the future to reason about the current state.
- It must be possible to run the project in three different modes: considering laser only, with considering radar only, or with using both sensors.
- For every mode, the overall RMSE (2d position only) may not be more than 10% increased to what the original solution is able to reach (this number depends on the individual measurement sequence)
- The RMSE of laser AND radar must be lower than radar only or laser only
- The NIS of radar measurements must be between 0.35 and 7.81 in at least 80% of all radar update steps.
## PROJECT GRADING  
- I recommend a hall of fame for the lowest overall RMSE using laser AND radar.
- I recommend to ask students to improve the initialization procedure and evaluate the RMSE during the first 20 steps.
## Project Rubric  
### 1. Compiling  
#### 1.1 Your code should compile.  
Compiled successfully.  
### 2. Accuracy  
#### 2.2 px, py, vx, vy output coordinates must have an RMSE <= [.09, .10, .40, .30] when using the file: "obj_pose-laser-radar-synthetic-input.txt", which is the same data file the simulator uses for Dataset 1.  
Meet.
### 3. Follows the Correct Algorithm  
#### 3.1 Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.  
My Kalman Filter implementation is completed at [`ukf.cpp`](./src/ukf.cpp)  
#### 3.2 Your Kalman Filter algorithm handles the first measurements appropriately.  
The first measurement is handled at [`ukf.cpp` Line128-194](./src/ukf.cpp#L128-L194).  
#### 3.3 Your Kalman Filter algorithm first predicts then updates.  
My Kalman Filter predict function will be called at [`ukf.cpp` Line195-207](./src/ukf.cpp#L195-L207)  
My Kalman Filter update functions will be called based on different measurements after calling prediction function at [`ukf.cpp` Line209-223](./src/ukf.cpp#L209-L223).  
#### 3.4 Your Kalman Filter can handle radar and lidar measurements.  
My Kalman Filter update functions will be called based on different measurements after calling prediction function. For laser measurement, the update function implementation can be found at [`ukf.cpp` Line401-555](./src/ukf.cpp#L401-L555). For radar measurement, the update function implementation can be found at [`ukf.cpp` Line557-710](./src/ukf.cpp#L557-L710).  
### 4. Code Efficiency  
#### 4.1 Your algorithm should avoid unnecessary calculations.  
Yes.  
## Code Style  
Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).
