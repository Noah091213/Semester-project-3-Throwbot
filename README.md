# Semester-project-3-Throwbot

This is a project about making a UR5 robot throw a ball and hit a target.

## Table of Contents

- [About the Project](#about-the-project)
- [Getting Started](#getting-started)
- [Usage](#usage)

## About the Project

This is the entire software framework that can identify a target using a camera, calculate a trajectory to said target, and use that trajectory to throw a ball at the target using a UR5 robot arm. It is developed to work specifically with a asler acA 1440-220uc camera, in the given robot cell, and a UR5. The target is a small, coloured velcro target, and the balls have the coresponding velcro glued on to stick when hitting the target.

Due to safety constraints of the robot, the math needs to account for a lot of extra constraints, and a 3d printed pillar was used to allow picking up the ball, as the gripper cannot reach the table with the required safety constraints.

## Getting Started

- Copy the github repo to a linux enviroment.
- In CMake, edit the paths to Matlabs libraries, this cannot be done automatically as it depends on the users Matlab installation
- Open the build folder and run cmake, then make
- In the shell script "run.sh" included in the build folder, edit the path to Matlab, to match the user setup
- Run the compiled program via the shell script

### Prerequisites

To run the program a few external libraries and programs need to be downloaded and installed.

Matlab for linux must be installed, this can be found on Matlabs own website: https://se.mathworks.com/help/install/ug/install-products-with-internet-connection.html

Basler Pylon is a required library to enable connection with the camera, the library can be found on Baslers own website:
https://docs.baslerweb.com/software-installation-(linux)

UR-RTDE is a library developed by professors on SDU and is the required library for communication and control with the UR5. It can be found on their Gitlab: https://sdurobotics.gitlab.io/ur_rtde/

### Installation

Step-by-step guide on how to install the project. 

```bash
# Example to download and run
git clone https://github.com/Noah091213/Semester-project-3-Throwbot
nano CMakeLists.txt
# Edit the path to the Matlab installation folder, in the 2 highlighted sets
# Save and exit CMakeLists.txt
cd build
cmake ..
make
nano run.sh
# Edit path to Matlab folder again, this is necessary to actual run the program with the correct ICU
# Save and exit run.sh

# Profit!
```


## Usage

To run the program use the following:

```bash
# To run the program
cd build
./run.sh
# Profit!
```
