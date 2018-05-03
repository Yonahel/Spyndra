# Simulation
-----
This folder contains packages for Spyndra's simulation in Gazebo.

To run them, first copy all the packages to catkin workspace, and then run catkin_make
```
cp * ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
``` 

Then you may need to install gazebo_ros_pkgs for joint controllers

```
sudo apt-get install ros-indigo-gazebo-ros-pkgs ros-indigo-gazebo-ros-control ros-indigo-ros-control ros-indigo-ros-controllers
```

If there is problem related to downloading models in Gazebo 2, download the models from http://models.gazebosim.org/ and put them into ~/.gazebo/models

```
wget -r -R "index\.html*" http://models.gazebosim.org/
mkdir ~/.gazebo/models
cp -r models.gazebosim.org/* ~/.gazebo/models
```

Then you can directly run Spyndra in Gazebo

```
roslaunch spyndra_gazebo spyndra_world.launch
```

Or go to the machine learning folder and run the machine learning algorithms.