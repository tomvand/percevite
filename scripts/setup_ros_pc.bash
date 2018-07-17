#!/bin/bash

echo "Set up ROS for multiple computers with Ubuntu connection sharing..."
export ROS_MASTER_URI=http://192.168.45.1:11311
export ROS_IP=192.168.45.1
export ROS_HOSTNAME=`hostname`.local
echo "ROS settings:"
export | grep ROS

echo "Waiting for /pose..."
rostopic echo /pose -n 1

echo "Send message to slamdunk"
rostopic pub /test std_msgs/String "Hello SLAMdunk!" -1
echo "Done."

