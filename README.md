# PETcat_localization
Localization and avoidance modules for PETCat, the long term project.

## Contribution Guidelines

### 1. Fork the repo on GitHub: Use the button at the top right.

### 2. Clone the project to your own machine

``` git clone https://github.com/${your_username_here}/PETcat_localization ```

### 3. Commit changes to your own branch: 

Create a new branch by

``` git checkout -b ${your_branch_name} ```

### 4. Push your work back up to your fork: 

Navigate to the top-level repo directory and:
``` 
git add .
git commit -m "Explanative commit message"
git push origin ${your_branch_name} 
```
    
### 5. Submit a Pull request so that we can review your changes:

Create a new pull request from the `Pull Requests` tab on this repo, not the fork.

Request reviews from at least two people. 
  
---

## Overview

For localisation we decided to implement SLAM ( Simultaneous Localisation and Mapping ). For this we started off with looking into [OpenVSLAM](https://openvslam.readthedocs.io/en/master/ros_package.html).

Some courses for conceptual understanding we found helpful:

1. [Mobile-robotics](http://ais.informatik.uni-freiburg.de/teaching/ss19/robotics/)
2. [Visual Navigation](https://vision.in.tum.de/teaching/ss2013/visnav2013)

### Dependencies:

1. [Eigen](http://eigen.tuxfamily.org/)
2. [g2o](https://github.com/RainerKuemmerle/g2o)
3. [DBoW2](https://github.com/shinsumicco/DBoW2)
4. [OpenCV](https://opencv.org/)
5. [Pangolin](https://github.com/stevenlovegrove/Pangolin)

### Issues:

* Mismatch between OpenCV versions : 

Resolved by properly setting up paths and making changes in CMakefiles to enable it to run on required version.

* Video streaming from gazebo :

To make it work with live video feed from gazebo we had to redirect some topics and attach a camera plugin on our model which was kept in the simulation environment with obstalces and walls around it.

Testing OpenVSLAM with a drone moving through a virtual alley:
[Click here for video](https://drive.google.com/file/d/1Y_Hu-au4z2JEkjzrlUlrZw7EljRF2b_A/view)


To test this on a quadruped we tried to stimulate the model we had using [towr](http://docs.ros.org/lunar/api/towr/html/index.html) and [towr_ros](https://github.com/ethz-adrl/towr)

We had to modify the code to make our own node to publish the final pose for the quadruped to reach.

![Image](https://github.com/isro01/PETcat_localization/blob/document/img/towr_code.png)

[Demo video can be found here](https://drive.google.com/open?id=1MN2vm95H1Dhfs8n3qlgd3xLlkyX3Nfzf
)

* Improper friction values for joints:

We didnt have proper friction values for the robot joints and hence the random motion in the demo video.

Moved on to a [repository](https://github.com/chvmp/champ) to implement the model with SLAM. 

---

## Benchmarking

Had to benchmark against [OpenVSLAM](https://openvslam.readthedocs.io/en/master/installation.html) and [GMapping](http://wiki.ros.org/gmapping) the following factors:

1. Feature detection
2. Detection rate
3. CPU usage and GPU information
4. Odometry drift

The work done for the same can be found [here](https://github.com/m2kulkarni/PETcat_Benchmark)

---
