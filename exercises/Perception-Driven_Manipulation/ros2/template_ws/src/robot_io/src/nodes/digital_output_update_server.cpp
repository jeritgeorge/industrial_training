/*
 * ouput_state_service_server.cpp
 *
 *  Created on: Dec 14, 2012
 *      Author: jnicho
 */

#include "rclcpp/rclcpp.hpp"
#include <robot_io/srv/DigitalOutputUpdate.h>
#include <soem_beckhoff_drivers/msg/DigitalMsg.h>

static const std::string OUTPUT_TOPIC = "/digital_outputs";
static const std::string OUTPUT_SERVICE = "/digital_output_update";
static const std::string SERVER_RUNNING_PARAM = "/digital_output_server_running";

class DigitalOutputUpdateServer : public rclcpp::Node
{
public:

	DigitalOutputUpdateServer() : Node("digital_output_update_server")
	{
		std::string nodeName = this->get_name();

	    RCLCPP_INFO_STREAM(this->get_logger(), nodeName<<": Grasp execution action node started");
		pub_ = this->create_publisher<soem_beckhoff_drivers::msg::DigitalMsg>(OUTPUT_TOPIC, 1);

		while ( pub_.getNumSubscribers() <= 0 && rclcpp::ok())
		{
			rclcpp::Duration(5.0).sleep();
		    RCLCPP_INFO_STREAM(this->get_logger(), nodeName<<": Waiting for digital output subscribers");
		}

		// initializing and sending output message
		output_msg_.values.resize(robot_io::srv::DigitalOutputUpdate::Request::COUNT, false);
		output_msg_.values[robot_io::srv::DigitalOutputUpdate::Request::COLLISION] = true;
		pub_->publish(output_msg_);
	    RCLCPP_INFO_STREAM(this->get_logger(), nodeName<<": Turning on air pressure to collision sensor");

		server_ = this->create_server<(OUTPUT_SERVICE,&DigitalOutputUpdateServer::serviceCallback,this);
		RCLCPP_INFO_STREAM(this->get_logger(), nodeName<<": Advertising "<<OUTPUT_SERVICE<<" service");

		this->set_parameter(SERVER_RUNNING_PARAM,true);
	}

	~DigitalOutputUpdateServer()
	{
		// set param to false
		bool serverRunning = false;
		if(this->get_parameter(SERVER_RUNNING_PARAM,serverRunning) && serverRunning)
		{
			this->set_parameter(SERVER_RUNNING_PARAM,false);
		}
	}

	bool serviceCallback(const std::shared_ptr<robot_io::DigitalOutputUpdate::Request> req,
						std::shared_ptr<robot_io::DigitalOutputUpdate::Response> res)
	{
		if(validateRequest(req))
		{
			output_msg_.values[req->bit_index] = (bool)req->output_bit_state;
			pub_.publish(output_msg_);

			rclcpp::Duration(0.5f).sleep();
			res->output_bit_array = output_msg_.values;
			return true;
		}
		else
		{
			RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name()<<": Rejected output update request");
			return false;
		}
	}

	static bool checkSingletonInstanceRunning()
	{
		bool server_running = false;
		this->get_parameter(SERVER_RUNNING_PARAM,server_running);
		return server_running;
	}

protected:

	  // ros comm
	  rclcpp::Service<robot_io::srv::DigitalOutputUpdate>::SharedPtr server_;
	  rclcpp::Publisher<soem_beckhoff_drivers::msg::DigitalMsg>::SharedPtr pub_;

	  // messages
	  soem_beckhoff_drivers::msg::DigitalMsg output_msg_;

	  bool validateRequest(const std::shared_ptr<robot_io::DigitalOutputUpdate::Request> req)
	  {
		  if(req->bit_index >= req->COUNT|| req.bit_index == req->COLLISION)
		  {
			  return false;
		  }

		  return true;
	  }

};

int main(int argc,char** argv)
{
	ros::init(argc,argv,"digital_output_update_server");
	ros::NodeHandle nh("/"); // will use global namespaces

	if(DigitalOutputUpdateServer::checkSingletonInstanceRunning())
	{
		ROS_WARN_STREAM(ros::this_node::getName()<<": Server is already running, only one instance is allowed");
		return 0;
	}

	DigitalOutputUpdateServer server;
	server.init();
	ros::spin();

	return 0;
}
