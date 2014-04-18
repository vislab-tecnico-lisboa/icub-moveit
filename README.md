# icub-moveit

URDF iCub model ready for moveit! in ROS Hydro

## Environment

All the functionality was tested on machines configured with:

+ Ubuntu 12.04 LTS
+ ROS Hydro (install it [here](http://wiki.ros.org/hydro/Installation/Ubuntu))

## Download and Setup

Open a terminal and navigate to the folder where you want to download the package.

Now, run:

    git clone https://github.com/vislab-tecnico-lisboa/icub-moveit.git

After completing the download, you need to add the new folder to the $ROS_PACKAGE_PATH, run:

    gedit ~/.bashrc

At the end of the file, add the following line:

    export ROS_PACKAGE_PATH=[Path to the downloaded folder]:$ROS_PACKAGE_PATH
    
Save the file and run:

    source .bashrc

## Compilation

Note that you may get some errors while compiling saying that you do not have some dependencies from other ROS packages installed. In that case, you will need to find what packages you are missing and then install them. For example, I got this message:

    [rospack] Error: package/stack 'icub_controller_interface' depends on non-existent package 'robot_mechanism_controllers' and rosdep claims that it is not a system dependency. Check the ROS_PACKAGE_PATH or try calling 'rosdep update'

To solve this I have installed this package:

    sudo apt-get install ros-hydro-robot-mechanism-controllers

Before running the modules for the first time complete the following instructions to compile the packages:

    rosmake yarp_msgs bond_core
    rosmake icub_ros
    rosmake icub_description
    rosmake icub_moveit
    rosmake icub_controller_interface
