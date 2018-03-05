# Installation instructions
1. Set up SVO2 install workspace according to the SVO2 manual. 
2. Clone this repository. 
3. Navigate to the src directory 
    1. Copy rpg_svo_example here 
    2. Clone okvis_ros (https://github.com/ethz-asl/okvis_ros.git) here 
    3. Go into okvis_ros and manually clone okvis (https://github.com/ethz-asl/okvis.git ) 
4. Go back to percevite_ws and source svo_install_ws/install/setup.bash so that this workspace will be chained. 
5. Compile: 
    1. Export ARM_ARCHITECTURE=armv7l 
    2. Make a swap file as compilation consumes too much RAM:
      ```
      dd if=/dev/zero of=swap.img bs=1024k count=1000 
      mkswap swap.img 
      sudo swapon swap.img 
      ```
    3. Compile with catkin_make. This will take a while (1h or more). 
    4. Remove the swap file: 
      ```
      sudo swapoff swap.img 
      rm swap.img 
      ```

Before use, source percevite_ws/setup.bash (or percevite_ws/devel/setup.bash). 
