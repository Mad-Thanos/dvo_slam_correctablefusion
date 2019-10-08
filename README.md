# Dense Visual SLAM for RGB-D Cameras (dvo_slam)

This repository contains a [DVO-SLAM fork](https://jsturm.de/publications/data/kerl13iros.pdf) (ROS Kinetic) with modifications for our BMVC 2017 paper "Efficient Online Surface Correction for Real-time Large-Scale 3D Reconstruction".

 *  **dvo_core**
    Core implementation of the motion estimation algorithm. 
    
 *  **dvo_ros**
    Integration of *dvo_core* with ROS.
    
 *  **dvo_slam**
    Pose graph SLAM system based on *dvo_core* and integration with ROS.
    
 *  **dvo_benchmark**
    Integration of *dvo_slam* with TUM RGB-D benchmark, see http://vision.in.tum.de/data/datasets/rgbd-dataset.
    
## Installation

Setup ROS environment and create a ROS catkin workspace:
```
# source ROS environment
source /opt/ros/kinetic/setup.bash

# create ROS workspace folder
mkdir ros_catkin_ws
cd ros_catkin_ws

# create catkin workspace and source setup .sh file
catkin_make
source devel/setup.bash
```

Checkout the DVO-SLAM source code:
```
cd src/
git clone https://github.com/robmaier/dvo_slam_vhkf.git
```

```
# build it using catkin_make
catkin_make
```

## Usage

Estimating the camera trajectory from an RGB-D image stream:

*TODO*

For visualization:

 *  Start RVIZ
 *  Set the *Target Frame* to `/world`
 *  Add an *Interactive Marker* display and set its *Update Topic* to `/dvo_vis/update`
 *  Add a *PointCloud2* display and set its *Topic* to `/dvo_vis/cloud`

The red camera shows the current camera position. The blue camera displays the initial camera position.

## Publications

The following publications describe the approach:

 *   **Dense Visual SLAM for RGB-D Cameras** (C. Kerl, J. Sturm, D. Cremers), In Proc. of the Int. Conf. on Intelligent Robot Systems (IROS), 2013.
 *   **Robust Odometry Estimation for RGB-D Cameras** (C. Kerl, J. Sturm, D. Cremers), In Proc. of the IEEE Int. Conf. on Robotics and Automation (ICRA), 2013
 *   **Real-Time Visual Odometry from Dense RGB-D Images** (F. Steinbruecker, J. Sturm, D. Cremers), In Workshop on Live Dense Reconstruction with Moving Cameras at the Intl. Conf. on Computer Vision (ICCV), 2011.

## License

The packages *dvo_core*, *dvo_ros*, *dvo_slam*, and *dvo_benchmark* are licensed under the GNU General Public License Version 3 (GPLv3), see http://www.gnu.org/licenses/gpl.html.

The package *sophus* is licensed under the MIT License, see http://opensource.org/licenses/MIT.
