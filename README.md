hrpsyspy --port 2809 --robot MIDJAXON
hcf.init("MIDJAXON")
roslaunch hrpsys_ros_bridge_jvrc midjaxon_vision_connect.launch 
roslaunch hrpsys_ros_bridge_jvrc midjaxon_ros_bridge_choreonoid.launch
