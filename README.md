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
