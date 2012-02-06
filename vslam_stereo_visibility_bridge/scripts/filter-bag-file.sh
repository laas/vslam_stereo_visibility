#!/bin/sh
#
# This takes any bag file as input and filter it to keep only the data
# used by the SLAM simu.launch file.
#
# This makes data publication much more reactive.
#
# See launch/simu.launch for the required topics list.

if test x"$1" = x; then
 echo "first argument must be the bag file to be filtered"
 exit
fi
rosbag filter "$1" filtered.bag "topic == '/wide/left/image_raw' or topic == '/wide/left/camera_info' or topic == '/wide/right/image_raw' or topic == '/wide/right/camera_info' or topic == '/plan_left_ankle' or topic =='/joint_states' or topic =='/dynamic_graph/base_link' or topic =='/dynamic_graph/base_footprint' or topic =='/dynamic_graph/com' or topic =='/dynamic_graph/zmp'"
