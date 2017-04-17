hku_m100_gazebo
=========

A ROS package for simulating the DJI Matrice 100 Drone in Gazebo


**Installation**

Clone this repo into your catkin workspace and build.


**Configuration**
  * Turn on DJI Matrice 100 and connect it to the PC simulator
  * Turn on the Manifold and setup the roscore on it
  * Set ROS_MASTER_URI to the Manifold IP


**Usage**
Open the gazebo simulation
> $ roslaunch hku_m100_gazebo empty_world_with_tags.launch

You should see in the gazebo world a parking Matrice 100 and a truck

![Gazebo world](/images/gazebo.png)

Connect the drone in Gazebo with the drone in the PC simulator
> $ rosrun hku_m100_gazebo hku_m100_pcsim_gazebo_bridge

Control the drone using the DJI remote controller, you should see the drone flying in the gazebo world

![Drone connection](/images/gazebo_drone_in_control.png)

You can also play with the gimbal

![Drone connection](/images/gazebo_gimbal.png)
