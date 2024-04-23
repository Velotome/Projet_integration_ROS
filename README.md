# Projet_integration_ROS

## Environment setup

Clone the repo 

Setup environment
```bash
cd catkin_ws
. /opt/ros/noetic/setup.bash
catkin build
. devel/setup.bash
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