hrpsyspy --port 2809 --robot MIDJAXON
hcf.init("MIDJAXON")
rosrun tf static_transform_publisher 0.0 0.0.0 0.0 0.0 0.0 0.0 map WAIST_LINK 100
/usr/bin/choreonoid /home/yosuke/catkin_ws/src/rtm-ros-robotics/rtmros_choreonoid/hrpsys_ros_bridge_jvrc/config/midjaxon_pdO1.cnoid --start-simulation
roslaunch hrpsys_ros_bridge_jvrc midjaxon_vision_connect.launch 
roslaunch hrpsys_ros_bridge_jvrc midjaxon_ros_bridge_choreonoid.launch
roslaunch hrpsys_ros_bridge_jvrc midjaxon_jvrc.launch 
