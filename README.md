# Sensor Fusion of 2D Lidar and Odometry using Particle Filter [C++ & Python]

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

### Start launch file

```
roslaunch odom-lidar-pf turtlebot.launch
```

### Run PF node [Run either CPP OR Python]

For CPP

```
rosrun odom-lidar-pf mcl_cpp
```

For Python

```
rosrun odom-lidar-pf mcl_python.py
```

### Run Turtlebot3 controller

```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
