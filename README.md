# Turtlebot Autonomous Exploration (3D) 
# Modifications by Scott

## Overview

This is an ROS implementation of infomation-theoretic exploration using [turtlebot](http://wiki.ros.org/Robots/TurtleBot) with a RGBD camera (e.g. Kinect). It is designed for autonomous mapping of indoor office-like environments (flat terrain). All the computation is performed on the turtlebot laptop and intermediate results can be viewed from remote PC. The output consist of both 2D and 3D [Octomap](http://octomap.github.io/) (.ot) file and saved on the turtlebot laptop.
Link to [wiki page](http://wiki.ros.org/turtlebot_exploration_3d) (where you can find a video example.).


If you find this package useful, please consider citing the follow paper:

* S. Bai, J. Wang, F. Chen, and B. Englot, "Information-Theoretic Exploration with Bayesian Optimization," IEEE/RSJ International Conference on Intelligent Robots and Systems(IROS), October 2016. [PDF](http://personal.stevens.edu/~benglot/Bai_Wang_Chen_Englot_IROS2016_AcceptedVersion.pdf)


## How do I get set up? 


with great difficult 


### Running:

From Turtlebot:
```
$ roslaunch exploration_2d sim.launch
$ roslaunch exploration_2d exploration_rviz.launch
$ rosrun exploration_2d exploration_2d
```

### Authors ###

Shi Bai, Xiangyu Xu.
[RFAL (Robust Field Autonomy Lab)](http://personal.stevens.edu/~benglot/index.html), Stevens Institute of Technology.
