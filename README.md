# Double SLS Controller

## Install/Getting Started  
### 1. Install PX4-Autopilot v1.13.3  
```
git clone --branch v1.13.3 https://github.com/PX4/PX4-Autopilot.git --recursive
cd ~/PX4-Autopilot
git submodule update
cd ~
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd ~/PX4-Autopilot
make px4_sitl gazebo
```
### 2. Install ROS Noetic & MAVROS  
Follow the guide at https://docs.px4.io/v1.13/en/ros/mavros_installation.html  
### 3. Install Gazebo Link Attacher  
```
cd catkin_ws/src  
git clone https://github.com/pal-robotics/gazebo_ros_link_attacher.git  
cd ..
catkin build
```
### 4. Install Double SLS Controller
```
cd ~/catkin_ws/src
git clone https://github.com/ANCL/double_sls_controller.git --recursive
cd ..
catkin build
```
### 5. Move the Link Attacher Script
```
sudo mv ~/catkin_ws/src/double_sls_controller/scripts/attach_sls.py ~/catkin_ws/src/gazebo_ros_link_attacher/scripts/attach_sls.py
```   
### 6. Modify /.bashrc
```
sudo vim ~/.bashrc
```  
add following contents:
```
# For PX4v1.13.3
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/PX4-Autopilot/Tools/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/sitl_gazebo
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/double_sls_controller/models
```
## Run DSLS SITL
### 1. Launch PX4 SITL
```
# in a new terminal
roslaunch double_sls_controller double_px4vision_sls_world.launch
```  
### 2. Attach the Drones and the Slung Load
```
# in a new terminal
rosrun gazebo_ros_link_attacher attach_sls.py
```
### 3. Run QGroundControl
```
# in a new terminal
cd
./QGroundControl.AppImage
```
### 4. Launch Double SLS Contoller
```
# in a new terminal
roslaunch double_sls_controller double_sls_controller.launch
```
