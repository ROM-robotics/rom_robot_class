#### install


```diff
echo "export BOBO_MODEL=robot1" >> ~/.bashrc
echo "#export BOBO_MODEL=sim_robot1" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=99" >> ~/.bashrc
```
```
sudo apt install -y ros-humble-gazebo-ros* ros-humble-backward-ros
sudo apt install -y ros-humble-ros2-control*
sudo apt install -y ros-humble-controller-*
sudo apt install -y ros-humble-slam-toolbox ros-humble-robot-localization ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3* ros-humble-twist-mux ros-humble-nav2*
sudo apt install -y ros-humble-gazebo-ros2-control \
ros-humble-ros2controlcli \
ros-humble-ros2-control \
ros-humble-ros2-controllers \
ros-humble-effort-controllers \
ros-humble-velocity-controllers \
ros-humble-gripper-controllers \
ros-humble-position-controllers \
ros-humble-tricycle-controller \
ros-humble-admittance-controller \
ros-humble-forward-command-controller \
ros-humble-joint-trajectory-controller \
ros-humble-rqt-joint-trajectory-controller \
ros-humble-controller-manager \
ros-humble-rqt-controller-manager \
ros-humble-hardware-interface \
ros-humble-controller-interface \
ros-humble-transmission-interface \
ros-humble-control-msgs \
ros-humble-controller-manager-msgs \
ros-humble-realtime-tools \
ros-humble-ros2-control-test-assets \
ros-humble-joint-limits \
ros-humble-joint-state-broadcaster \
ros-humble-force-torque-sensor-broadcaster \
ros-humble-force-torque-sensor-broadcaster \
ros-humble-ackermann-* \
ros-humble-rplidar-ros \
ros-humble-imu-sensor-broadcaster \
ros-humble-rmw-cyclonedds-cpp \
ros-humble-rplidar-ros \
libserial-dev \
openssh-server 
```