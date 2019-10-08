# Dense Visual SLAM for RGB-D Cameras (dvo_slam)

This repository contains a [DVO-SLAM](https://jsturm.de/publications/data/kerl13iros.pdf) fork (ROS Kinetic) with modifications for our [BMVC 2017 paper "Efficient Online Surface Correction for Real-time Large-Scale 3D Reconstruction"](http://www.rmaier.net/pub/maier2017efficient.pdf).

## Packages
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

Checkout and build the DVO-SLAM source code:
```
# clone repository
cd src/
git clone https://github.com/robmaier/dvo_slam_vhkf.git

# create TUM benchmark output directory
cd dvo_slam_vhkf/dvo_benchmark
mkdir dvo_slam_vhkf/dvo_benchmark/output/
cd ..

# build package (and workspace) using catkin_make
catkin_make
```

## Dataset

Download one of the [TUM RGB-D Benchmark sequences](https://vision.in.tum.de/data/datasets/rgbd-dataset/download):
```
# go into data folder
cd src/dvo_slam_vhkf/data/
# download dataset
wget https://vision.in.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_long_office_household.tgz
# extract data
tar -xvzf rgbd_dataset_freiburg3_long_office_household.tgz
mv rgbd_dataset_freiburg3_long_office_household fr3_office
# associate color and depth images using their timestamps
python associate.py fr3_office/rgb.txt fr3_office/depth.txt > fr3_office/assoc.txt
# go back to parent folder
cd ../../../
```


## Usage

Run ```roscore``` (in a new different terminal):
```
source /opt/ros/kinetic/setup.bash
roscore
```

Estimate camera trajectory for the downloaded RGB-D dataset:
```
roslaunch dvo_benchmark benchmark.launch dataset:=$PWD/src/dvo_slam_vhkf/data/fr3_office
```

We calculate the camera tracking accuracy and plot the differences between the groundtruth trajectory and the estimated trajectory as follows:
```
cd src/dvo_slam_vhkf/data/fr3_office
python ../evaluate_ate.py groundtruth.txt ../../dvo_benchmark/output/trajectory.txt --plot plot.png --verbose
```

You can also use RVIZ for real-time visualization:
```
source /opt/ros/kinetic/setup.bash

# start RVIZ
rosrun rviz rviz
```
In the GUI, 
 *  Start RVIZ
 *  Set the *Fixed Frame* (Global Options) to `/world`
 *  Add an *Interactive Marker* display and set its *Update Topic* to `/dvo_vis/update`
 *  Optional: add a *PointCloud2* display and set its *Topic* to `/dvo_vis/cloud`

## Publications

The following publications describe the approach:
 *   [**Dense Visual SLAM for RGB-D Cameras**](https://vision.in.tum.de/_media/spezial/bib/kerl13iros.pdf) (C. Kerl, J. Sturm, D. Cremers), In Proc. of the Int. Conf. on Intelligent Robot Systems (IROS), 2013.
 *   [**Robust Odometry Estimation for RGB-D Cameras**](https://vision.in.tum.de/_media/spezial/bib/kerl13icra.pdf) (C. Kerl, J. Sturm, D. Cremers), In Proc. of the IEEE Int. Conf. on Robotics and Automation (ICRA), 2013
 *   [**Real-Time Visual Odometry from Dense RGB-D Images**](https://vision.in.tum.de/_media/spezial/bib/steinbruecker_sturm_cremers_iccv11.pdf) (F. Steinbruecker, J. Sturm, D. Cremers), In Workshop on Live Dense Reconstruction with Moving Cameras at the Intl. Conf. on Computer Vision (ICCV), 2011.

## License

The packages *dvo_core*, *dvo_ros*, *dvo_slam*, and *dvo_benchmark* are licensed under the GNU General Public License Version 3 (GPLv3), see http://www.gnu.org/licenses/gpl.html.
