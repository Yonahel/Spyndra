# Spyndra ROS
This project provides software architecture and controller in ROS for the open-source robotic platform <a href="http://www.creativemachineslab.com/spyndra.html">Spyndra</a>.

## Installation
This project assumes using <a href="http://wiki.ros.org/indigo/Installation/Ubuntu">ROS Indigo</a>.

1. Suppose you have not created your ROS workspace, you can create one by the command:
```
$ source /opt/ros/lunar/setup.bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

2. Go to the workspace and download the repository.
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/dannyhung1128/spyndra.git
```

3. Make sure you have ros packages.
```
$ sudo apt-get install ros-indigo-ros-control ros-indigo-ros-controllers
$ sudo apt-get install ros-indigo-gazebo-ros-control ros-indigo-effort-controllers ros-indigo-joint-state-controller ros-indigo-joint-trajectory-controller
```

4. Check ROS dependencies.
```
$ rosdep update
$ rosdep check --from-paths src --ignore-src --rosdistro indigo
```
   If any dependency is missing, you can auto-install them by the command
```
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro indigo -y
```

5. Verify installation bin usage.
```
$ source ~/catkin_ws/devel/setup.bash
```

6. Build the source code.
```
$ cd ~/catkin_ws
$ catkin_make
```

After that, you are all set for this project.

## Usage
1. First, start the ros core by the command:
```
$ roscore
```

2. Start running all the nodes
```
$ rosrun spyndra bno055_node.py
$ rosrun spyndra motor_control_node.py
$ rosrun spyndra spyndra_node.py
$ rosrun spyndra user.py
```

3. Press number "1", "2", "3", "4" under terminal screen that is running "user.py" node to send motor command to spyndra. (Now it's still under development, so the command type maybe changed)

	1. "1" is for "cmd_1": standing Gait
	2. "2" is for "cmd_2": random Gait
	3. "3" is for "cmd_3": not yet assigned
    4. "4" is for "cmd_4": not yet assigned

4. To check current motor signals sent to Spyndra by motor_control_node:
```
rostopic echo /motor_signal
```

5. To check current IMU data published by bno055_node:
```
rostopic echo /imu/data
```

6. IMU data will be automatically saved in "~/catkin_ws/src/spyndra/src/" with name "imu_data+time.bag" is ros bag format.

## License

This work is licensed under <a href="https://opensource.org/licenses/MIT">MIT License</a>.