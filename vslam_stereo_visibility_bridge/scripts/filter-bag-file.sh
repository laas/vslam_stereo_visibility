#!/bin/sh
#
# This takes any bag file as input and filter it to keep only the data
# used by the SLAM simu.launch file.
#
# This makes data publication much more reactive.
#
# See launch/simu.launch for the required topics list.
"
if x"$1" == x; then
 echo "first argument must be the bag file to be filtered"
 exit
fi
rosbag filter $1 slam_$1 "topic == '/wide/left/image_raw' or topic == '/wide/left/camera_info' or topic == '/wide/right/image_raw' or topic == '/wide/right/camera_info' or topic == '/plan_left_ankle' or topic =='/joint_states'"
