#include "movella_dot/movella_dot.hpp"

using namespace std;
using namespace std::literals::string_literals;
//--------------------------------------------------------------------------------
// Class implementation
//--------------------------------------------------------------------------------
namespace movella_dot
{
using std::placeholders::_1;
using namespace std::chrono_literals;

/// @brief Class constructor. Initializes xdpcHandler and sets up publishers for each DOT.
/// @param node_name Node name [String]
/// @param xdpc_period xdpcHandler period [double, in miliseconds]
MovellaDot::MovellaDot(std::string& node_name, double xdpc_period) : Node(node_name)
{
	// XDPC Code - Initializes, Scans for DOTS, connects to available DOTs.
	if (!xdpcHandler_initialize())
		return;

	// ROS 2 Publishers setup

	timer_ = this->create_wall_timer((xdpc_period_)ms, std::bind(&MovellaDot::timer_callback, this));

	rclcpp::PublisherOptions pub_options;
	pub_options.qos_overriding_options =
			rclcpp::QosOverridingOptions::with_default_policies();	// Allow overriding QoS settings (history, depth,
																															// reliability)

	int numdots = 0;
	for (auto& device : xdpcHandler.connectedDots())
	{
		// Create IMU publisher and append to list
		topic = "/dot_"s + std::itoa(numdots + 1) + "/IMU"s;
		auto pubI = create_publisher<sensor_msgs::msg::imu>(topic, rclcpp::QoS(100), pub_options);
		imuPub_.push_back(pubI);

		// Create magnetic field publisher and append to list
		std::string topic = "/dot_"s + std::itoa(numdots + 1) + "/magnetic_field"s;
		auto pubM = create_publisher<sensor_msgs::msg::magnetic_field>(topic, rclcpp::QoS(100), pub_options);
		magPub_.push_back(pubM);

		// Create DOT data publisher and append to list
		topic = "/dot_"s + std::itoa(numdots + 1) + "/dot"s;
		auto pubD = create_publisher<movella_dot_msgs::msg::dot_sensor>(topic, rclcpp::QoS(100), pub_options);
		dotPub_.push_back(pubD);

		// Set device filter and payload profiles
		auto filterProfiles = device->getAvailableFilterProfiles();
		RCLCPP_INFO_STREAM(get_logger(), std::itoa(filterProfiles.size()) + " available filter profiles: ");
		for (auto& f : filterProfiles)
			RCLCPP_INFO_STREAM(get_logger(), f.label());

		RCLCPP_INFO_STREAM(get_logger(), "Current profile: "s + device->onboardFilterProfile().label());
		if (device->setOnboardFilterProfile(XsString("General")))
			RCLCPP_INFO_STREAM(get_logger(), "Successfully set filter profile to General.");
		else
			RCLCPP_ERROR_STREAM(get_logger(), "Setting filter profile failed!");

		// Some payload modes from the API, for convenience:
		//	HighFidelitywMag: SampleTimeFine, dq, dv, Angular velocity, Acceleration, Magnetic Field, Status
		//	ExtendedQuaternion: SampleTimeFine, Orientation (Quaternion), Free acceleration, Status
		//	ExtendedEuler: SampleTimeFine, Orientation (Euler angles), Free acceleration, Status
		//	CustomMode4: SampleTimeFine, Orientation (Quaternion), dq, dv, Angular velocity, Acceleration, Magnetic Field,
		// Status
		RCLCPP_INFO_STREAM(get_logger(), "Putting device into measurement mode.");
		if (!device->startMeasurement(XsPayloadMode::CustomMode4))
		{
			RCLCPP_INFO_STREAM(get_logger(),
												 "Could not put device into measurement mode. Reason: "s + device->lastResultText());
			continue;
		}

		numdots++;	// increase iterator
	}
	RCLCPP_INFO_STREAM(get_logger(), "Main loop. Publishing data to ROS 2 topics");

	// First printing some headers so we see which data belongs to which device
	// for (auto const& device : xdpcHandler.connectedDots())
	// 	cout << setw(42) << left << device->bluetoothAddress();
	// cout << endl;

	// this->t0_ = XsTime::timeStampNow();
};

/// @brief Destructor. Finalizes xdpcHandler, stops measurement.
MovellaDot::~MovellaDot()
{
	for (auto const& device : xdpcHandler.connectedDots())
	{
		RCLCPP_INFO_STREAM(get_logger(), "Resetting heading to default for device %s :", device->bluetoothAddress());
		if (device->resetOrientation(XRM_DefaultAlignment))
			RCLCPP_INFO_STREAM(get_logger(), "OK");
		else
			RCLCPP_INFO_STREAM(get_logger(), "NOK: %s" << device->lastResultText());
	}
	RCLCPP_INFO_STREAM(get_logger(), "Stopping measurement...");
	for (auto device : xdpcHandler.connectedDots())
	{
		if (!device->stopMeasurement())
			RCLCPP_INFO_STREAM(get_logger(), "Failed to stop measurement.");

		if (!device->disableLogging())
			RCLCPP_INFO_STREAM(get_logger(), "Failed to disable logging.");
	}

	xdpcHandler.cleanup();
};

/// @brief Timer callback function.
/// Periodically checks xdpc handler for new incoming data and prepares messages for publication.
///
void timer_callback()
{
	RCLCPP_INFO(this->get_logger(), "Polling DOT sensors for publishing...");

	if (xdpcHandler_.packetsAvailable())
	{
		// cout << "\r";
		int device_num = -1;
		for (auto const& device : xdpcHandler_.connectedDots())
		{
			device_num++;
			// Retrieve a packet from the device
			XsDataPacket packet = xdpcHandler_.getNextPacket(device->bluetoothAddress());

			std::string segment, seglist;
			std::stringstream test(device->bluetoothAddress().c_str());
			while (std::getline(test, segment, ':'))
			{
				seglist = segment;
			}

			if (packet.containsStatus())	// Filling in (legacy?) status
			{
				uint32_t st = packet.status();
				dotMsgs_[device_num].status = st;
			}
			if (packet.containsSampleTimeFine())	// Sample time fine
			{
				uint32_t stf = packet.sampleTimeFine();

				dotMsgs_[device_num].sample_time_fine = st;
			}
			if (packet.containsVelocityIncrement())	 // Sample time fine
			{
				XsVector dv = packet.velocityIncrement();

				dotMsgs_[device_num].delta_velocity.x = dv.x();
				dotMsgs_[device_num].delta_velocity.y = dv.y();
				dotMsgs_[device_num].delta_velocity.z = dv.z();
			}
			if (packet.containsOrientation())	 // Filling in orientation data
			{
				XsQuaternion quat = packet.orientationQuaternion();

				// Fills in quaternion data
				dotMsgs_[device_num].quaternion.x = quat.x();
				dotMsgs_[device_num].quaternion.y = quat.y();
				dotMsgs_[device_num].quaternion.z = quat.z();
				dotMsgs_[device_num].quaternion.w = quat.w();	 // Real part

				imuMsgs_[device_num].quaternion.x = quat.x();
				imuMsgs_[device_num].quaternion.y = quat.y();
				imuMsgs_[device_num].quaternion.z = quat.z();
				imuMsgs_[device_num].quaternion.w = quat.w();	 // Real part
			}
			if (packet.containsOrientationIncrement())	// Quaternion rate
			{
				XsQuaternion dq = packet.orientationIncrement();

				// Fills in quaternion data
				dotMsgs_[device_num].delta_quaternion.x = dq.x();
				dotMsgs_[device_num].delta_quaternion.y = dq.y();
				dotMsgs_[device_num].delta_quaternion.z = dq.z();
				dotMsgs_[device_num].delta_quaternion.w = dq.w();	 // Real part
			}
			if (packet.containsRawAcceleration())	 // Filling in acceleration data
			{
				XsVector acc = packet.rawAccelerationConverted();

				imuMsgs_[device_num].linear_acceleration.x = acc.x();
				imuMsgs_[device_num].linear_acceleration.y = acc.y();
				imuMsgs_[device_num].linear_acceleration.z = acc.z();
			}
			if (packet.containsRawGyroscopeData())	// Filling in gyrocope data
			{
				// Note: there is a rawGyroscopeTemperatureData, which _also_ returns a vector.
				// I'm not sure how the temperature is used with that call; does it contain compensated gyro signals or not?

				XsVector gyro = packet.rawGyroscopeDataConverted();

				imuMsgs_[device_num].angular_velocity.x = gyro.x();
				imuMsgs_[device_num].angular_velocity.y = gyro.y();
				imuMsgs_[device_num].angular_velocity.z = gyro.z();
			}
			if (packet.containsRawMagneticField())	// Filling in magnetometer data
			{
				XsVector mag = packet.rawMagneticFieldConverted();

				magMsgs_[device_num].magnetic_field.x = mag.x();
				magMsgs_[device_num].magnetic_field.y = mag.y();
				magMsgs_[device_num].magnetic_field.z = mag.z();
			}

			// Triggers publishers
			imuMsgs_[device_num].header.stamp = this->get_clock()->now();
			imuMsgs_[device_num].header.frame_id = device->deviceTagName();
			imuPub_[device_num].publish(imuMsgs_[device_num]);

			magMsgs_[device_num].header.stamp = this->get_clock()->now();
			magMsgs_[device_num].header.frame_id = device->deviceTagName();
			magPub_[device_num].publish(magMsgs_[device_num]);

			dotMsgs_[device_num].header.stamp = this->get_clock()->now();
			dotMsgs_[device_num].header.frame_id = device->deviceTagName();
			dotPub_[device_num].publish(dotMsgs_[device_num]);
		}
		// cout << flush;
	}
	XsTime::msleep(0);
};
}	 // namespace movella_dot

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);

	// Setup executor for multiple nodes
	//  movella_dot::DotSensorMsg dot_msg;

	// bool orientationResetDone = false;
	// int64_t startTime = XsTime::timeStampNow();

	// Spin all nodes using executor
	// executor.spin();

	rclcpp::spin(std::make_shared<MovellaDot::MovellaDot>(10.0));
	rclcpp::shutdown();
	return 0;
}
