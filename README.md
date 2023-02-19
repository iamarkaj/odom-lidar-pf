# INSTRUCTIONS

### Install Turtlebot3

```
cd ~/catkin_ws/src/
git clone https://github.com/iamarkaj/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
cd ~/catkin_ws && rosdep install --from-paths src --ignore-src -r -y && catkin_make
```

### Install odom-lidar-pf

```
cd ~/catkin_ws/src/
git clone https://github.com/iamarkaj/odom-lidar-pf.git
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/odom-lidar-pf/models" >> ~/.bashrc
cd ~/catkin_ws && rosdep install --from-paths src --ignore-src && catkin_make
```

### Run odom-lidar-pf

```
roslaunch odom-lidar-pf turtlebot.launch
```

### Run Turtlebot3 controller

```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
