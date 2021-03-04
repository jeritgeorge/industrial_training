/*
 * OpenCloseGraspActionServer.cpp
 *
 *  Created on: Apr 19, 2013
 *      Author: jnicho
 */

/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
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


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <object_manipulation_msgs/msg/GraspHandPostureExecutionAction.h>
#include <object_manipulation_msgs/msg/GraspHandPostureExecutionGoal.h>
#include <robot_io/srv/DigitalOutputUpdate.h>
#include <soem_beckhoff_drivers/msg/DigitalMsg.h>

using namespace object_manipulation_msgs;
using namespace actionlib;


typedef robot_io::srv::DigitalOutputUpdate::Request DigitalOutputType;

static const std::string OUTPUT_TOPIC = "/digital_outputs";
static const std::string INPUT_TOPIC = "/digital_inputs";
static const std::string OUTPUT_SERVICE = "/digital_output_update";

class SimpleGraspActionServer : pubic rclcpp::Node
{
private:
  typedef ActionServer<GraspHandPostureExecutionAction> GEAS;
  typedef GEAS::GoalHandle GoalHandle;

public:
  SimpleGraspActionServer() : Node("grasp_execution_action_node"),
	use_sensor_feedback_(false),
   suction_on_output_channel_(DigitalOutputType::SUCTION1_ON),
   suction_check_input_channel_(DigitalOutputType::SUCTION1_ON)
  {
	this->action_server_ = rclcpp_action::create_server<GraspHandPostureExecutionAction>
	(this, "grasp_execution_action",
	std::bind(&GraspExecutionAction::goalCB, this, _1),
	std::bind(&GraspExecutionAction::cancelCB, this, _1));
  }

  ~SimpleGraspActionServer()
  {
  }

  void init()
  {
	    std::string nodeName = this->get_name();

	    // service client
	    service_client_ = this->create_client<robot_io::srv::DigitalOutputUpdate>(OUTPUT_SERVICE);
	    while(!service_client_->wait_for_service(1s))
	    {
			if (!rclcpp::ok())
			{
				RCLCPP_ERROR_STREAM(this->get_logger(), nodeName<<": Waiting for "<<OUTPUT_SERVICE<<" to start");
			}
			RCLCPP_INFO_STREAM(this->get_logger(), "service not available, waiting again...");
	    }

	    if(!fetchParameters() )
	    {
	    	RCLCPP_ERROR_STREAM(this->get_logger(), nodeName<<": Did not find required ros parameters, exiting");
	    	rclcpp::shutdown();
	    	return;
	    }

	    if(!validateChannelIndices())
	    {
	    	RCLCPP_ERROR_STREAM(this->get_logger(), nodeName<<": One or more parameter values are invalid");
	    	rclcpp::shutdown();
	    	return;
	    }

	    RCLCPP_INFO_STREAM(this->get_logger(), nodeName<<": Grasp execution action node started");
  }

private:


  void goalCB(GoalHandle gh)
  {
    std::string nodeName = this->get_name();

    RCLCPP_INFO(this->get_logger(), "%s",(nodeName + ": Received grasping goal").c_str());

    robot_io::srv::DigitalOutputUpdate::Request req;
    robot_io::srv::DigitalOutputUpdate::Response res;
    bool success;

	switch(gh.getGoal()->goal)
	{
		case GraspHandPostureExecutionGoal::PRE_GRASP:

			gh.setAccepted();
			RCLCPP_INFO_STREAM(this->get_logger(), nodeName + ": Pre-grasp command accepted");

			req.bit_index = suction_on_output_channel_;
			req.output_bit_state = true;

			if(service_client_.call(req,res))
			{
				gh.setSucceeded();
				RCLCPP_INFO_STREAM(this->get_logger(), nodeName + ": Pre-grasp command succeeded");
			}
			else
			{
				gh.setAborted();
				RCLCPP_INFO_STREAM(this->get_logger(), nodeName + ": Pre-grasp command aborted");
			}


			break;

		case GraspHandPostureExecutionGoal::GRASP:

			gh.setAccepted();
			RCLCPP_INFO_STREAM(this->get_logger(), nodeName + ": Grasp command accepted");

			req.bit_index = suction_on_output_channel_;
			req.output_bit_state = false;
			success = service_client_.call(req,res);

			if(success)
			{
				if(use_sensor_feedback_ && !checkSensorState())
				{
					gh.setAborted();
					RCLCPP_INFO_STREAM(this->get_logger(), nodeName + ": Grasp command aborted");
					break;
				}
			}
			else
			{
				gh.setAborted();
				RCLCPP_INFO_STREAM(this->get_logger(), nodeName + ": Grasp command aborted");
				break;
			}

			gh.setSucceeded();
			RCLCPP_INFO_STREAM(this->get_logger(), nodeName + ": Grasp command succeeded");
			break;

		case GraspHandPostureExecutionGoal::RELEASE:

			gh.setAccepted();
			RCLCPP_INFO_STREAM(this->get_logger(), nodeName + ": Release command accepted");

			req.bit_index = suction_on_output_channel_;
			req.output_bit_state = true;

			if(service_client_.call(req,res))
			{
				gh.setSucceeded();
				RCLCPP_INFO_STREAM(this->get_logger(), nodeName + ": Release command succeeded");
			}
			else
			{
				gh.setAborted();
				RCLCPP_INFO_STREAM(this->get_logger(), nodeName + ": Release command aborted");
			}

			break;

		default:

			RCLCPP_WARN_STREAM(this->get_logger(),nodeName + ": Unidentified grasp request, rejecting request");
			gh.setRejected();
			break;
	}

  }

  void cancelCB(GoalHandle gh)
  {
    std::string nodeName = ros::this_node::getName();
	RCLCPP_INFO_STREAM(this->get_logger(), nodeName + ": Canceling current grasp action");
    gh.setCanceled();
    RCLCPP_INFO_STREAM(this->get_logger(), nodeName + ": Current grasp action has been canceled");
  }

  bool fetchParameters()
  {
	  ros::NodeHandle nh;

	  bool success = true;
	  nh.getParam("use_sensor_feedback",use_sensor_feedback_);
	  success = success && nh.getParam("suction_on_output_channel",suction_on_output_channel_);
	  success = success && nh.getParam("suction_check_output_channel",suction_check_input_channel_);
	  return success;
  }

  bool validateChannelIndices()
  {
	  typedef robot_io::DigitalOutputUpdate::Request DigitalOutputType;

	  if(suction_on_output_channel_ >= (int)DigitalOutputType::COUNT || suction_on_output_channel_ == (int)DigitalOutputType::COLLISION)
	  {
		  return false;
	  }

	  if(suction_check_input_channel_ >= int(DigitalOutputType::COUNT))
	  {
		  return false;
	  }

	  return true;
  }

  bool checkSensorState()
  {
	  ros::NodeHandle nh("/");
	  soem_beckhoff_drivers::DigitalMsg::ConstPtr input_msg_ =
			  ros::topic::waitForMessage<soem_beckhoff_drivers::DigitalMsg>(INPUT_TOPIC,nh,ros::Duration(2.0f));
	  if(!input_msg_)
	  {
		  ROS_ERROR_STREAM(ros::this_node::getName()<<": Input message received invalid");
		  return true;
	  }
	  else
	  {
		  return ((input_msg_->values[suction_check_input_channel_] ==  1) ? true : false);
	  }

  }

  // ros comm
  GEAS action_server_;
  rclcpp::Client<robot_io::srv::DigitalOutputUpdate>::SharedPtr service_client_;

  // ros parameters
  int suction_on_output_channel_; // index value to output channel for suction
  int suction_check_input_channel_; // index value to input channel for vacuum sensor
  bool use_sensor_feedback_;

};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SimpleGraspActionServer>());
	rclcpp::shutdown();
  return 0;
}




