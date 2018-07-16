Installation
============
SLAMDunk preparation
```
sudo ntpupdate pool.ntp.org
```

Clone git
```
cd
git clone https://github.com/tomvand/percevite_slamdunk.git percevite_ws
git submodule update --init --recursive
```

Install dependencies
```
sudo add-apt-repository -y ppa:paparazzi-uav/ppa
sudo apt-get update
sudo apt-get -f -y install paparazzi-dev
```

Fix [opencv4tegra / ROS version conflicts](http://wiki.ros.org/action/show/NvidiaJetson/TK1?action=show&redirect=NvidiaJetsonTK1)
```
cd ~/percevite_ws/scripts
./fix_opencv4tegra.sh
```

Set up pprzlink
```
cd ~/percevite_ws/src/percevite/ext/pprzlink
make PPRZLINK_LIB_VERSION=2.0 all pymessages
```

Build ROS package
```
cd ~/percevite_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
```

Launch
```
source devel/setup.bash
roslaunch percevite percevite.launch
```
The terminal should display output of the percevite node. Use ctrl+c to quit.


Optional steps
=============

Internet connection through PC
------------------------------
On the PC, run the `scripts/share_connection.sh` script to forward internet traffic to the SLAMDunk.

On the SLAMDunk, run `scripts/share_connection_client.sh` to set a default route through the pc. Replace `Tom-HP16.local` with the hostname of the PC.


Networking through Bebop2
-------------------------
On the Bebop2, use `dev` to make the firmware writable. Make a backup of `/bin/rndis_host_setup.sh` and replace it with `scripts/rndis_host_setup.sh` from this repo. The modified script sets a static IP for the Bebop2 (`192.168.45.28`) and enables IP forwarding.

On the SLAMDunk, add `sudo route add -net 192.168.42.0 netmask 255.255.255.0 gw 192.168.45.28 usb0` to `/etc/rc.local`.


ROS networking
--------------
On the SLAMDunk, source the `setup.bash` script in `percevite_ws/` to set the current time and `ROS_IP` and `ROS_HOSTNAME` environment variables.

On the PC, source the `scripts/setup_ros_pc.bash` to set the required ROS environment variables.


Run at startup
--------------
To run the percevite launchfile at startup, copy the `scripts/percevite.conf` file to `/etc/init` on the SLAMDunk. Ensure that the paths in the conf file are correct.
