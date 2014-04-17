/*
 * IcubRosInterface.h
 *
 *  Created on: Aug 1, 2013
 *      Author: rui
 */

#ifndef ICUBROSINTERFACE_H_
#define ICUBROSINTERFACE_H_
#include <ros/ros.h>
#include <yarp_msgs/JointValues.h>
#include <sensor_msgs/JointState.h>


const double PI = 3.14159265359;
const double DEG_TO_RAD = PI / 180;

class IcubRosInterface {
private:
	// The node handler
	ros::NodeHandle nh_;

	// The private node handler
	ros::NodeHandle n_priv_;

	// ROS yarp subscriber
	ros::Subscriber yarp_joints_;
	ros::Publisher ros_joints_;

	void icubRosCallback(const yarp_msgs::JointValues::ConstPtr& msg);


public:
	IcubRosInterface();

	IcubRosInterface(ros::NodeHandle & n);
	virtual ~IcubRosInterface();
};

#endif /* ICUBROSINTERFACE_H_ */
