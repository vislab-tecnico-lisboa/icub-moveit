/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Stuart Glaser

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
//#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/JointState.h>
#include <yarp_msgs/JointValues.h>
bool isEqual(const double & x, const double & y, const double & threshold)
{
	if(fabs(x-y)<threshold)
	{
		return true;
	}

	return false;
}


class JointTrajectoryExecuter
{
	protected:
		// The node handler
		ros::NodeHandle nh_;

		// The private node handler
		ros::NodeHandle n_priv;

		actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
		std::string action_name_;

	public:

		JointTrajectoryExecuter(std::string name) : as_(nh_, name, boost::bind(&JointTrajectoryExecuter::executeCB, this, _1), false)
		{
			using namespace XmlRpc;
			ros::NodeHandle pn("~");

			// Gets all of the joints
			/*XmlRpc::XmlRpcValue joint_names;
			if (!pn.getParam("/omnirob_arm_controller/joints", joint_names))
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
			joints_set_ = nh_.advertise<sensor_msgs::JointState>("/joints_set", 1); // Topic to set node values
			move_arm_trajectory_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/icub_arm_trajectory",1);
			trajectory_feedback_ = nh_.advertise<control_msgs::FollowJointTrajectoryActionFeedback>("/feedback",1);
			as_.start();
		}

		~JointTrajectoryExecuter()
		{
//			pub_controller_command_.shutdown();
//			sub_controller_state_.shutdown();
//			watchdog_timer_.stop();
		}

private:




		static bool setsEqual(const std::vector<std::string> &a, const std::vector<std::string> &b)
		{
			if (a.size() != b.size())
				return false;

			for (size_t i = 0; i < a.size(); ++i)
			{
				if (count(b.begin(), b.end(), a[i]) != 1)
					return false;
			}
			for (size_t i = 0; i < b.size(); ++i)
			{
				if (count(a.begin(), a.end(), b[i]) != 1)
					return false;
			}

			return true;
		}


		void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr & goal)
		{
			control_msgs::FollowJointTrajectoryResult result_;
			control_msgs::FollowJointTrajectoryActionFeedback feedback_;

			feedback_.feedback.error.positions.resize(goal->trajectory.points[0].positions.size());
			feedback_.feedback.actual.positions.resize(goal->trajectory.points[0].positions.size());
			feedback_.feedback.desired.positions.resize(goal->trajectory.points[0].positions.size());


			// For each trajectory joint state point...
			for(uint16_t tp=0; tp<goal->trajectory.points.size(); ++tp)
			{
//				std::cout << "tp: "<< tp <<" of: " << goal->trajectory.points.size() << std::endl;
				sensor_msgs::JointState joint_state;
				joint_state.name=goal->trajectory.joint_names;
				joint_state.position=goal->trajectory.points[tp].positions;
				joints_set_.publish(joint_state);
				// For each joint...
				for(uint16_t jp=0; jp<feedback_.feedback.error.positions.size(); ++jp)
				{
//					std::cout << "jp: "<< jp << " of: " << feedback_.feedback.error.positions.size() << std::endl;
					feedback_.feedback.error.positions[jp]=0.0;
					feedback_.feedback.actual.positions[jp]=goal->trajectory.points[tp].positions[jp];
					feedback_.feedback.desired.positions[jp]=goal->trajectory.points[tp].positions[jp];

				}
				trajectory_feedback_.publish(feedback_);

			}
			std::cout << "saiu" << std::endl;

			//move_arm_trajectory_.publish(goal->trajectory);

			//ros::Time time = robot_->getTime();
			//ros::Duration dt = time - last_time_;

			trajectory_msgs::JointTrajectoryPoint goal_trajectory_point=goal->trajectory.points.back();

			// wait while end position not reached
			ros::Rate r(1.0);

			double delta=0.009;
			int max_time_without_moving=5;
			success=true;
			ROS_INFO("Robot is going to goal...");

			// For each trajectory point...
			/*for(int16_t t=0; t<goal->trajectory.points.size(); ++t)
			{
				// Send trajectory point to robot
				for(int16_t p=0; p<goal->goal_tolerance)
			}*/

			// Checks that we have ended inside the goal tolerances
			/*bool inside_goal_constraints = true;
			for (size_t i = 0; i < joints_.size() && inside_goal_constraints; ++i)
			{
				if (goal->goal_tolerance[i])
			          inside_goal_constraints = false;
			}/*


			/*while(isGoingToGoal(goal_trajectory_point,delta))
			{
				int time_without_moving=0;
				double gamma=0.001;
				while(!asMoved(gamma))
				{
					std::cout << " didn't move (" <<time_without_moving<<","<<max_time_without_moving<<")"<< std::endl;
					time_without_moving++;
					if(time_without_moving>=max_time_without_moving)
					{
						// SOMETHING WENT TERRIBLY WRONG!!!!
						success=false;
						break;
					}
					r.sleep();
				}
				if(success)
				{
					ROS_INFO("Robot is still going to goal...");
				}
				else
				{
					break;
				}
			}*/

			//ROS_INFO("Done");
			success=true;
			if(success)
			{
		    result_.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
				// set the action state to succeeded
				as_.setSucceeded(result_);
				ROS_INFO("Succeeded");

			}
			else
			{
		        result_.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
				as_.setAborted(result_);
			}

		}

		bool isGoingToGoal(const trajectory_msgs::JointTrajectoryPoint & goal_trajectory_point)
		{
			bool is_not_in_goal=false;

			boost::shared_ptr<const sensor_msgs::JointState> current_robot_joint_values = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");

			/*for(unsigned int j=0; j < goal_trajectory_point.positions.size(); ++j)
			{
				std::cout <<"diff: "<< (double) fabs(current_robot_joint_values->position[j]-goal_trajectory_point.positions[j]) << " current: " << (double)current_robot_joint_values->position[j] << " goal: " << goal_trajectory_point.positions[j] <<" " << delta <<  std::endl;
				if(!isEqual(current_robot_joint_values->position[j],goal_trajectory_point.positions[j],delta))
					is_not_in_goal=true;
			}*/

			return is_not_in_goal;
		}

		bool asMoved(double & delta)
		{
			bool as_moved=false;

			boost::shared_ptr<const sensor_msgs::JointState> current_robot_joint_values_ptr = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");

			sensor_msgs::JointState current_robot_joint_values = *current_robot_joint_values_ptr;
			for(unsigned int j=0; j < old_robot_joint_values.position.size(); ++j)
			{

				std::cout <<"diff: "<< (double) fabs(current_robot_joint_values.position[j]-old_robot_joint_values.position[j]) << " current: " << (double)current_robot_joint_values.position[j] << " goal: " << old_robot_joint_values.position[j] <<" " << delta <<  std::endl;
				if(!isEqual(current_robot_joint_values.position[j],old_robot_joint_values.position[j],delta)) // robot moved
					as_moved=true;
			}

			old_robot_joint_values = current_robot_joint_values;
			return as_moved;
		}



		sensor_msgs::JointState old_robot_joint_values;

		bool success;
		ros::Publisher joints_set_;
		ros::Publisher move_arm_trajectory_;
		ros::Publisher trajectory_feedback_;
		trajectory_msgs::JointTrajectory current_traj_;


		std::vector<std::string> joint_names_;
		std::map<std::string,double> goal_constraints_;
		std::map<std::string,double> trajectory_constraints_;
		double goal_time_constraint_;
		double stopped_velocity_tolerance_;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "follow_joint_trajectory");
	ros::NodeHandle n;
	JointTrajectoryExecuter jte(ros::this_node::getName());
	ros::spin();

	return 0;
}
