#ifndef MOVELLA_DOT_HPP
#define MOVELLA_DOT_HPP

#include <iostream>
#include <iomanip>

#include "xdpchandler.h"

// ROS
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <sstream>
#include "movella_msgs/msg/dot_sensor.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
// #include <geometry_msgs/msg/quaternion.hpp>

namespace movella_dot
{
class MovellaDot : public rclcpp::Node
{
public:
	MovellaDot(std::string node_name);
	~MovellaDot(void);

	XdpcHandler xdpcHandler_;
	// std::vector<std::shared_ptr<XsDotDevice>> dot_publishers; // XsDotDevice, from the SDK

	bool xdpc_initialize()
	{
		if (!this->xdpcHandler_.initialize()){
			RCLCPP_ERROR_STREAM(get_logger(), "Failed to initialize xdpcHandler. Aborting.");
			return false;
		}

		this->xdpcHandler_.scanForDots();

		if (this->xdpcHandler_.detectedDots().empty())
		{
			RCLCPP_ERROR_STREAM(get_logger(), "No Movella DOT device(s) found. Aborting.");
			this->xdpcHandler_.cleanup();
			return false;
		}
		RCLCPP_INFO_STREAM(get_logger(),
											 "Detected "  << this->xdpcHandler_.detectedDots().size() << " DOT device(s).");

		this->xdpcHandler_.connectDots();

		if (this->xdpcHandler_.connectedDots().empty())
		{
			RCLCPP_ERROR_STREAM(get_logger(), "Could not connect to any Movella DOT device(s). Aborting.");
			this->xdpcHandler_.cleanup();
			return false;
		}
		RCLCPP_INFO_STREAM(get_logger(),
											 "Connected to " << this->xdpcHandler_.connectedDots().size() << " DOT device(s).");
		return true;
	};

private:
	int config;	 // Sensor configuration

	// ROS
	rclcpp::TimerBase::SharedPtr timer_;
	double t0_;
	std::chrono::milliseconds xdpc_period_;	 // xdpc callback period (ms)

	std::vector<rclcpp::Publisher<movella_msgs::msg::DotSensor>::SharedPtr> dotPub_;	 // DOT Message publisher
	std::vector<rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> imuPub_;					 // IMU(Quat,Gyro,Acc) measurements
	std::vector<rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr> magPub_;	// Magnetic Field components

	std::vector<movella_msgs::msg::DotSensor> dotMsgs_;		// DOT Messages
	std::vector<sensor_msgs::msg::Imu> imuMsgs_;						// IMU Messages
	std::vector<sensor_msgs::msg::MagneticField> magMsgs_;	// Magnetic Field Messages

	void timer_callback();	// Reads XSens DOT via handler and prepares messages for publishers, at an interval of
													// xdpc_period ms
};
}	 // namespace movella_dot
#endif