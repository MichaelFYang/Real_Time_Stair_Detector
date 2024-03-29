# A Real-Time Stair Detector #

## What is this repository for?

A light-weighted stair detection algorithm using 2D correlation analysis on real-time LiDAR scans that searches for a similar pattern of a manually generated kernel. 

[A video shows a result of this stair detector here.](https://youtu.be/617FaTN6klg)

## How to get the stair detector running on your workspace 

#### Copy this planner workspace
```bash
git clone https://github.com/MichaelFYang/Real_Time_Stair_Detector.git
cd Real_Time_Stair_Detector/
catkin build
source devel/setup.bash
```

#### Before launch the stair detector, make sure you have the correct topics of odom and registered poincloud in the config file
Open the config file as the way you like
``` bash
gedit <<YOUR WORKSPACE>>/src/stair_detector/config/default.yaml 
```

#### Launch the graph decoder
```bash
roslaunch stair_detector stair_detector.launch
```

#### Generate a 2D pattern kernel
```bash
python <<YOUR WORKSPACE>>/src/stair_detector/src/kernal_generation.py
```
The python script "kernel_generation.py" generates a normalized 2D kernel that used to find stairs. 
You can modify the kernel and use it to seach for the pattern you want from the real-time LiDAR scans.

## Reminder

This is a research project that strongly depends on and ties to the certain platforms and frameworks used by CMU-OSU team for DARPA SubT Challenge. It is not well generalized for integration into other platforms. Intentions of using this code for you own researches may require works of code modifications.

Questions asked in "Issues" for integrations will be answered with best availabilities. 


## Who could I talk to? 

Fan Yang
(michael.yfan24@gmail.com, fanyang1@ethz.ch, fanyang2@alumni.cmu.edu)
