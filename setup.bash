#!/bin/bash

echo "Setting up ROS..."

echo "Set time..."
sudo ntpdate pool.ntp.org

echo "Source percevite workspace"
source devel/setup.bash

echo "Set up ROS for multiple computers with Ubuntu connection sharing..."
export ROS_IP=`hostname -I | sed 's/ //'`
export ROS_HOSTNAME=`hostname`.local
echo "ROS settings:"
export | grep ROS

echo "Restart ROS and slamdunk node..."
sudo stop slamdunk_ros_node
sudo stop roscore
sudo start roscore
sudo start slamdunk_ros_node

echo "Waiting for /pose..."
rostopic echo /pose -n 1

echo "Wait for message from PC..."
echo "(Use Ctrl+C if no pc connected)"
rostopic echo /test -n 1
echo "Done."
