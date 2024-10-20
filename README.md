# ROS Integration project

The customer wanted to have a system capable of scanning a 3D object in the real world by using a depth camera and a 6-axis industrial robotic arm.
This project aims to answer this need by producing an accurate simulation of the environment. We simulate all the components of the system, namely a robotic arm, a depth camera and the obstacles around the robot.

To do this, we used ROS and the software it provides such as Moveit, Gazebo and Rviz.

## Environment setup

Clone the repo 
```bash
git clone https://github.com/Bp91230/Projet_integration_ROS.git
```

Setup environment
```bash
cd catkin_ws
. /opt/ros/noetic/setup.bash
catkin build
. devel/setup.bash
```
## Moveit Setup Assistant

```bash
roslaunch moveit_setup_assistant setup_assistant.launch
```
## Re-generate URDF from xacro

A version of the URDF is available in the repo from the start (catkin_ws/src/motoman_copy/motoman_hc10_support/urdf/model_hc10.urdf) but if modifications to the xacro files was made, you need to re-generate it. To do that execute the following command in a terminal from the catkin_ws folder.

```bash
rosrun xacro xacro src/motoman_copy/motoman_hc10_support/urdf/hc10.xacro -o src/motoman_copy/motoman_hc10_support/urdf/model_hc10.urdf
```

To generate the graph 
```bash
urdf_to_graphiz src/motoman_copy/motoman_hc10_support/urdf/model_hc10.urdf
```

## Launch demo in Rviz with MoveIt

```bash
roslaunch hc10_moveit_config demo.launch 
```

## Launch demo in Gazebo & Rviz

```bash
roslaunch hc10_moveit_config demo_gazebo.launch 
```
