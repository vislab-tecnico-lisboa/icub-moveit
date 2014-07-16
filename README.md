# icub-moveit

URDF iCub model ready for MoveIt! in ROS Hydro

## Environment

All the functionality was tested on machines configured with:

+ Ubuntu 12.04 LTS
+ ROS Hydro (install it [here](http://wiki.ros.org/hydro/Installation/Ubuntu))

## Download and Setup

Open a terminal and navigate to the folder where you want to download the package.

Now, run:

    git clone https://github.com/vislab-tecnico-lisboa/icub-moveit.git

After completing the download, you need to add the new folder to the $ROS_PACKAGE_PATH, run:

    sudo gedit ~/.bashrc

At the end of the file, add the following line:

    export ROS_PACKAGE_PATH=[Path to the downloaded folder]:$ROS_PACKAGE_PATH
    
Save the file and run:

    source ~/.bashrc

## Compilation

Note that you may get some errors while compiling saying that you do not have some dependencies (from other ROS packages) installed. In that case, you will need to find what packages you are missing and then install them. For example, I got this message:

    [rospack] Error: package/stack 'icub_controller_interface' depends on non-existent package 'robot_mechanism_controllers' and rosdep claims that it is not a system dependency. Check the ROS_PACKAGE_PATH or try calling 'rosdep update'

To solve this I have installed this package:

    sudo apt-get install ros-hydro-robot-mechanism-controllers

Before running the modules for the first time complete the following instructions to compile the packages:

    rosmake yarp_msgs bond_core
    rosmake icub_ros
    rosmake icub_description
    rosmake icub_controller_interface
    rosmake icub_moveit

## Running

At this point everything is set to successfuly run the modules. Although, you might need to install some missing ROS packages, for example, in my case, while installing ROS Hydro I have missed MoveIt! instalation so I had to install it this way:

    sudo apt-get install ros-hydro-moveit-*

Now, everything should work smoothly. Open a terminal, and run:

    roscore

Open another terminal, and run:

    roslaunch icub_moveit demo.launch

If you want, you can run the model without a graphical interface by running instead:

    roslaunch icub_moveit functional.launch

You can now plan trajectories for the iCub parts, set a start state, a goal state and hit plan.
Note that you will not be able to execute the trajectory since the main goal of this package is to be able to plan. You can connect this planner to the iCub simulator (or even the real robot) using [this other repository](https://github.com/vislab-tecnico-lisboa/yarp-with-moveit) which is responsible to connect the YARP iCub robot to this planning functionality implemented in ROS.
