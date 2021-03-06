Installation instructions for TX2 (JetPack 4.2)
===============================================

These instructions assume you are installing to `~`, change the `cd` instructions
to install to another folder.


Clone this repository
---------------------
```
cd <your code directory, assumed ~ for the rest of this readme>
git clone https://github.com/tomvand/percevite_slamdunk.git percevite_ws
git submodule update --init --recursive
```


Install dependencies
--------------------

Install ROS melodic
```
sudo apt install ros-melodic-desktop ros-melodic-image-proc
```

Compile Ivy-C (no binaries for Ubuntu 18 arm at the time of writing). Based on
https://wiki.paparazziuav.org/wiki/Installation/FromScratch#Ivy-c
```
sudo apt install subversion tcl-dev
cd
mkdir ivy-c
cd ivy-c
svn co https://svn.tls.cena.fr/svn/ivy/ivy-c/trunk
cd trunk/src
make
make install
sudo mv /usr/local/lib64/* /usr/local/lib
```

Install ocaml and dependencies
```
sudo apt install ocaml ocaml-findlib ocamlbuild
sudo apt install opam tk-dev
opam init
opam install xml-light
opam install ivy
source ~/.profile
```
We use `opam` for installation as the ocaml packages are not yet available through 
`apt`.


Compile/generate pprzlink
```
cd ~/percevite_ws/src/percevite/ext/pprzlink
make PPRZLINK_LIB_VERSION=2.0 all pymessages
```


Compile and run Percevite
-------------------------

Compile percevite
```
cd ~/percevite_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
```


Launch
```
source devel/setup.bash
roslaunch percevite percevite.launch
```









Old instructions for SLAMDunk (ignore)
======================================

Connect SLAMDunk to internet and set clock

On the PC, run the `scripts/share_connection.sh` script to forward internet traffic to the SLAMDunk. Find the hostname of the PC using `hostname`.

Connect to the SLAMDunk through ssh (e.g. `ssh slamdunk@192.168.45.1 -X`). Then run:
```
sudo route add default gw <PC hostname>.local
sudo ntpdate pool.ntp.org
```
Replace `<PC hostname>` with the hostname of the PC. Setting the clock is required to prevent certificate errors with git and to prevent clock skew when using make.

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
