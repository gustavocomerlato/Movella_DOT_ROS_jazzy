#ifndef MOVELLA_DOT_HPP
#define MOVELLA_DOT_HPP

#include <iostream>
#include <iomanip>

#include "xdpchandler.h"

// ROS
#include "rclcpp.hpp"
#include "movella_msgs/msg/dot_sensor_msg.hpp"
#include <string>
#include <vector>
#include <sstream>

namespace movella_dot
{
class MovellaDot : public rclcpp::Node
{
public:
	MovellaDot(std::string node_name, double xdpc_period);
	~MovellaDot(void);

	XdpcHandler xdpcHandler_;
	// std::vector<std::shared_ptr<XsDotDevice>> dot_publishers; // XsDotDevice, from the SDK

	bool xdpc_initialize()
	{
		if (!this->xdpcHandler_.initialize())
			RCLCPP_ERROR_STREAM(get_logger(), "Failed to initialize xdpcHandler. Aborting.");
		return 0;

		this->xdpcHandler_.scanForDots();

		if (this->xdpcHandler_.detectedDots().empty())
		{
			RCLCPP_ERROR_STREAM(get_logger(), "No Movella DOT device(s) found. Aborting.");
			this->xdpcHandler_.cleanup();
			return 0;
		}
		RCLCPP_INFO_STREAM(get_logger(),
											 "Detected " + std::itoa(this->xdpcHandler_.detectedDots().size()) + " DOT device(s)."s);

		this->xdpcHandler_.connectDots();

		if (this->xdpcHandler_.connectedDots().empty())
		{
			RCLCPP_ERROR_STREAM(get_logger(), "Could not connect to any Movella DOT device(s). Aborting.");
			this->xdpcHandler_.cleanup();
			return 0;
		}
		RCLCPP_INFO_STREAM(get_logger(),
											 "Connected to " + std::itoa(this->xdpcHandler_.connectedDots().size()) + " DOT device(s).");
		return 1;
	};

private:
	int config;	 // Sensor configuration

	// ROS
	rclcpp::TimerBase::SharedPtr timer_;
	double t0_;
	double xdpc_period;	 // xdpc callback period (ms)

	std::vector<rclcpp::Publisher<movella_msgs::msg::dot_sensor>::SharedPtr> dotPub_;	 // DOT Message publisher
	std::vector<rclcpp::Publisher<sensor_msgs::msg::imu>::SharedPtr> imuPub_;					 // IMU(Quat,Gyro,Acc) measurements
	std::vector<rclcpp::Publisher<sensor_msgs::msg::magnetic_field>::SharedPtr> magPub_;	// Magnetic Field components

	std::vector<movella_msgs::msg::dot_sensor> dotMsgs_;		// DOT Messages
	std::vector<sensor_msgs::msg::imu> imuMsgs_;						// IMU Messages
	std::vector<sensor_msgs::msg::magnetic_field> magPub_;	// Magnetic Field Messages

	void timer_callback();	// Reads XSens DOT via handler and prepares messages for publishers, at an interval of
													// xdpc_period ms
}
}	 // namespace movella_dot