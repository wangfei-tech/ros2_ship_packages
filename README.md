# The repo's statement
This repository mainly comes from https://github.com/OUXT-Polaris/ros_ship_packages. I will modify it to a ros2-based framework.
If you are interested in ros1, please refer to the source code！I will continue the simulation environment built by the original author. I am very grateful for their contribution.
# ros2_ship_packages
USV simulator for ROS2
# how to build（todo）
1.cd ~/catkin_ws/src
2.git@github.com:hakuturu583/ros_ship_packages.git
3.cd ../
4.sudo apt install ros-kinetic-jsk-*
5.ros-kinetic-hector-gazebo*
6.sudo apt install ros-kinetic-nmea*
7.sudo apt install ros-kinetic-velodyne*
8.sudo apt install ros-kinetic-nmea*
9.sudo apt install ros-kinetic-robot-pose-ekf
10.sudo apt install ros-kinetic-gps*
11.catkin_make
# robot model
## ros2_ship
![simulation on gazebo](https://github.com/hakuturu583/ros_ship_packages/blob/master/images/gazebo.png)
![simulation result on rviz](https://github.com/hakuturu583/ros_ship_packages/blob/master/images/rviz.png)

### simulated sensors
camera,VLP-16

### how to launch


## WAM-V
![simulation on gazebo](https://github.com/hakuturu583/ros_ship_packages/blob/master/images/wam-v_gazebo.png)
![simulation result on rviz](https://github.com/hakuturu583/ros_ship_packages/blob/master/images/wam-v_rviz.png)
[demo](https://www.youtube.com/watch?v=tQ_12pDbhCQ&feature=youtu.be)
[buoy recognition demo](https://youtu.be/tgicLday-1E)

### simulated sensors
gps,imu,camera,VLP-16,laser(2D)

### how to launch
roslaunch ros_ship_gazebo_plugins wam-v_gazebo.launch

# packages
## ros2_ship_control
ROS2 package for control simulated USV

## ros2_ship_description
ROS2 package for ros_ship_description(urdf,xacro,mesh,world files,etc..)

## ros2_ship_gazebo_plugins
gazebo plugins for simlated USV
#### simple_buoyancy_plugin
gazebo plugin for calculating buoyancy in a very simple way.
#### world_frame_publisher_plugin
gazebo plugin for broadcast gazebo_world frame.
#### simple_driving_force_plugin
gazebo plugin for calculating driving force from joint speed and ship speed

## ros2_ship_msgs
messages for ros_ship_packages

## ros2_ship_navigation
navigation package for ros_ship

## ros2_ship_recognition
recognition package for ros_ship
### object_recognition_node
object recognition node by using PCL(point cloud library)
set rosparams like below
![sample yaml file](https://github.com/hakuturu583/ros_ship_packages/blob/master/images/ros_ship_recognition_yaml.png)
stl and mesh file must be puted on (ros_ship_packages)/data/

## ros2_ship_visualization
#### gps_plotter.py
plot gps data from /fix(sensor_msgs/NavSatFix) and publish map image via google static map API
S is start point of ship
C is current position of the ship
![visualization in google map](https://github.com/hakuturu583/ros_ship_packages/blob/master/images/map_image_2.png)
#### kml_plotter.py
plot gps data from /fix(sensor_msgs/NavSatFix) topic in kml format
![visualize simulated gps data in google earth](https://github.com/hakuturu583/ros_ship_packages/blob/master/images/kml-plotter-node.png)
#### plot_characteristic_curve.py
plot characteristic curve  in .eps and .jpeg format
![plotted characteristic curve](https://github.com/hakuturu583/ros_ship_packages/blob/master/images/characteristic_curve.jpg)
