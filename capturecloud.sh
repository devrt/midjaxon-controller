#!/bin/sh

# ref http://answers.ros.org/question/215257/publish-rtab-point-cloud-from-rviz/

rosrun pcl_ros pointcloud_to_pcd input:=/multisense/organized_image_points2_color
#$ pcl_viewer ###.pcd
