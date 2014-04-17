/*
 * IcubRosInterface.cpp
 *
 *  Created on: Aug 1, 2013
 *      Author: rui
 */

#include "IcubRosInterface.h"

IcubRosInterface::IcubRosInterface()
{}

IcubRosInterface::IcubRosInterface(ros::NodeHandle & n): nh_(n), n_priv_("~")
{
	// Gets all of the joints
	/*XmlRpc::XmlRpcValue joint_names;
	if (!nh_.getParam("controller_list", joint_names))
				{
					ROS_FATAL("No joints given. (namespace: %s)", pn.getNamespace().c_str());
					exit(1);
				}
				if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
				{
					ROS_FATAL("Malformed joint specification.  (namespace: %s)", pn.getNamespace().c_str());
					exit(1);
				}
				for (int i = 0; i < joint_names.size(); ++i)
				{
					XmlRpcValue &name_value = joint_names[i];
					if (name_value.getType() != XmlRpcValue::TypeString)
					{
						ROS_FATAL("Array of joint names should contain all strings.  (namespace: %s)", pn.getNamespace().c_str());
						exit(1);
					}

					joint_names_.push_back((std::string)name_value);
				}

				pn.param("constraints/goal_time", goal_time_constraint_, 0.0);

				// Gets the constraints for each joint.
				for (size_t i = 0; i < joint_names_.size(); ++i)
				{
					std::string ns = std::string("constraints/") + joint_names_[i];
					double g, t;
					pn.param(ns + "/goal", g, -1.0);
					pn.param(ns + "/trajectory", t, -1.0);
					goal_constraints_[joint_names_[i]] = g;
					trajectory_constraints_[joint_names_[i]] = t;
				}
				pn.param("constraints/stopped_velocity_tolerance", stopped_velocity_tolerance_, 0.01);
		*/




	yarp_joints_ = nh_.subscribe("yarp_joint_values",1, &IcubRosInterface::icubRosCallback, this);
	ros_joints_ = nh_.advertise<sensor_msgs::JointState>("ros_joint_state", 1);
}

IcubRosInterface::~IcubRosInterface()
{
	// TODO Auto-generated destructor stub
}

void IcubRosInterface::icubRosCallback(const yarp_msgs::JointValues::ConstPtr& msg)
{
	sensor_msgs::JointState joint_state;

	int size_martelo=7;

	std::vector<std::string> joint_names;
	joint_names.push_back("raj1");
	joint_names.push_back("raj2");
	joint_names.push_back("raj3");
	joint_names.push_back("raj4");
	joint_names.push_back("raj5");
	joint_names.push_back("raj6");
	joint_names.push_back("right_wrist_yaw");

	for(uint16_t i=0; i < size_martelo; ++i)
	{
		joint_state.position.push_back(msg->v[i]*DEG_TO_RAD);
		joint_state.name=joint_names;
	}

	ros_joints_.publish(joint_state);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "icub_ros_interface");
	ros::NodeHandle n;

	IcubRosInterface icub_ros_interface(n);

	ros::spin();
}
