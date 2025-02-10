#include "../include/movella_dot.hpp"

using namespace std;
using namespace std::literals::string_literals;
//--------------------------------------------------------------------------------
// Class implementation
//--------------------------------------------------------------------------------
namespace movella_dot
{
	using std::placeholders::_1;
	using namespace std::chrono_literals;
	using namespace std::string_literals;

	/// @brief Class constructor. Initializes xdpcHandler_ and sets up publishers for each DOT.
	/// @param node_name Node name [String]
	/// @param xdpc_period xdpcHandler_ period [double, in miliseconds]
	MovellaDot::MovellaDot(std::string node_name) : Node(node_name)
	{
		// XDPC Code - Initializes, Scans for DOTS, connects to available DOTs.
		if (!xdpc_initialize()){
			RCLCPP_ERROR_STREAM(get_logger(), "Error: Failed to initialize XDPC Handler component.");
			return;
		}
		xdpc_period_ = 100ms;

		// ROS 2 Publishers setup

		timer_ = rclcpp::create_timer(this, this->get_clock(), xdpc_period_, std::bind(&MovellaDot::timer_callback, this));

		rclcpp::PublisherOptions pub_options;
		pub_options.qos_overriding_options =
				rclcpp::QosOverridingOptions::with_default_policies();	// Allow overriding QoS settings (history, depth,
																																// reliability)

		int numdots = 0;
		for (auto& device : xdpcHandler_.connectedDots())
		{
			// Create IMU publisher and append to list
			std::string topic = "/dot_"s + std::to_string(numdots + 1) + "/IMU"s;
			auto pubI = create_publisher<sensor_msgs::msg::Imu>(topic, rclcpp::QoS(100), pub_options);
			imuPub_.push_back(pubI);

			// Create magnetic field publisher and append to list
			topic = "/dot_"s + std::to_string(numdots + 1) + "/magnetic_field"s;
			auto pubM = create_publisher<sensor_msgs::msg::MagneticField>(topic, rclcpp::QoS(100), pub_options);
			magPub_.push_back(pubM);

			// Create DOT data publisher and append to list
			topic = "/dot_"s + std::to_string(numdots + 1) + "/dot"s;
			auto pubD = create_publisher<movella_msgs::msg::DotSensor>(topic, rclcpp::QoS(100), pub_options);
			dotPub_.push_back(pubD);

			// Set device filter and payload profiles
			auto filterProfiles = device->getAvailableFilterProfiles();
			RCLCPP_INFO_STREAM(get_logger(), std::to_string(filterProfiles.size()) + " available filter profiles: ");
			for (auto& f : filterProfiles)
				RCLCPP_INFO_STREAM(get_logger(), f.label());

			RCLCPP_INFO_STREAM(get_logger(), "Current profile: " << device->onboardFilterProfile().label());
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
													"Could not put device into measurement mode. Reason: " << device->lastResultText());
				continue;
			}

			numdots++;	// increase iterator
		}
		RCLCPP_INFO_STREAM(get_logger(), "Found " << numdots << " DOT sensors!");
		dotMsgs_.reserve(numdots);
		imuMsgs_.reserve(numdots);
		magMsgs_.reserve(numdots);
		// RCLCPP_INFO_STREAM(get_logger(), "Main loop. Publishing data to ROS 2 topics");

		// First printing some headers so we see which data belongs to which device
		// for (auto const& device : xdpcHandler_.connectedDots())
		// 	cout << setw(42) << left << device->bluetoothAddress();
		// cout << endl;

		// this->t0_ = XsTime::timeStampNow();
	};

	/// @brief Destructor. Finalizes xdpcHandler_, stops measurement.
	MovellaDot::~MovellaDot()
	{
		for (auto const& device : xdpcHandler_.connectedDots())
		{
			RCLCPP_INFO_STREAM(get_logger(), "Resetting heading to default for device "<< device->bluetoothAddress());
			if (device->resetOrientation(XRM_DefaultAlignment))
				RCLCPP_INFO_STREAM(get_logger(), "OK");
			else
				RCLCPP_INFO_STREAM(get_logger(), "NOK: %s" << device->lastResultText());
		}
		RCLCPP_INFO_STREAM(get_logger(), "Stopping measurement...");
		for (auto device : xdpcHandler_.connectedDots())
		{
			if (!device->stopMeasurement())
				RCLCPP_INFO_STREAM(get_logger(), "Failed to stop measurement.");

			if (!device->disableLogging())
				RCLCPP_INFO_STREAM(get_logger(), "Failed to disable logging.");
		}

		xdpcHandler_.cleanup();
	};

	/// @brief Timer callback function.
	/// Periodically checks xdpc handler for new incoming data and prepares messages for publication.
	///
	void MovellaDot::timer_callback(void)
	{
		RCLCPP_INFO_STREAM(this->get_logger(), "Polling DOT sensors for publishing...");

		if (xdpcHandler_.packetsAvailable())
		{
			RCLCPP_INFO_STREAM(this->get_logger(), "Available packets detected!");
			// cout << "\r";
			int device_num = -1;
			for (auto const& device : xdpcHandler_.connectedDots())
			{
				device_num++;
				// Retrieve a packet from the device

				RCLCPP_INFO_STREAM(this->get_logger(), "Getting packet from device " << device_num);
				XsDataPacket packet = xdpcHandler_.getNextPacket(device->bluetoothAddress());


				// RCLCPP_INFO_STREAM(this->get_logger(), "Setting segment and seglist");
				// std::string segment, seglist;
				// std::stringstream test(device->bluetoothAddress().c_str());
				// while (std::getline(test, segment, ':'))
				// {
				// 	seglist = segment;
				// }


				if (packet.containsStatus())	// Filling in (legacy?) status
				{
					RCLCPP_INFO_STREAM(this->get_logger(), "Got status!");
					uint32_t st = packet.status();

					dotMsgs_[device_num].status = st;
				}
				if (packet.containsSampleTimeFine())	// Sample time fine
				{

					RCLCPP_INFO_STREAM(this->get_logger(), "Got sampleTimeFine!");
					uint32_t stf = packet.sampleTimeFine();

					dotMsgs_[device_num].sample_time_fine = stf;
				}
				if (packet.containsVelocityIncrement())	 // Sample time fine
				{
					RCLCPP_INFO_STREAM(this->get_logger(), "Got velocityIncrement!");
					XsVector dv = packet.velocityIncrement();

					dotMsgs_[device_num].delta_velocity.x = dv[0];
					dotMsgs_[device_num].delta_velocity.y = dv[1];
					dotMsgs_[device_num].delta_velocity.z = dv[2];
				}
				if (packet.containsOrientation())	 // Filling in orientation data
				{
					RCLCPP_INFO_STREAM(this->get_logger(), "Got quaternion!");
					XsQuaternion quat = packet.orientationQuaternion();

					// Fills in quaternion data
					dotMsgs_[device_num].quaternion.x = quat[0];
					dotMsgs_[device_num].quaternion.y = quat[1];
					dotMsgs_[device_num].quaternion.z = quat[2];
					dotMsgs_[device_num].quaternion.w = quat[3];	 // Real part

					imuMsgs_[device_num].orientation.x = quat[0];
					imuMsgs_[device_num].orientation.y = quat[1];
					imuMsgs_[device_num].orientation.z = quat[2];
					imuMsgs_[device_num].orientation.w = quat[3];	 // Real part
				}
				if (packet.containsOrientationIncrement())	// Quaternion rate
				{
					RCLCPP_INFO_STREAM(this->get_logger(), "Got orientationIncrement!");
					XsQuaternion dq = packet.orientationIncrement();

					// Fills in quaternion data
					dotMsgs_[device_num].delta_quaternion.x = dq[0];
					dotMsgs_[device_num].delta_quaternion.y = dq[1];
					dotMsgs_[device_num].delta_quaternion.z = dq[2];
					dotMsgs_[device_num].delta_quaternion.w = dq[3];	 // Real part
				}
				if (packet.containsCalibratedAcceleration())	 // Filling in acceleration data
				{
					RCLCPP_INFO_STREAM(this->get_logger(), "Got calibratedAcceleration!");
					XsVector acc = packet.calibratedAcceleration();

					imuMsgs_[device_num].linear_acceleration.x = acc[0];
					imuMsgs_[device_num].linear_acceleration.y = acc[1];
					imuMsgs_[device_num].linear_acceleration.z = acc[2];
					
					dotMsgs_[device_num].linear_acceleration.x = acc[0];
					dotMsgs_[device_num].linear_acceleration.y = acc[1];
					dotMsgs_[device_num].linear_acceleration.z = acc[2];
				}
				if (packet.containsCalibratedGyroscopeData())	// Filling in gyrocope data
				{
					// Note: there is a rawGyroscopeTemperatureData, which _also_ returns a vector.
					// I'm not sure how the temperature is used with that call; does it contain compensated gyro signals or not?

					RCLCPP_INFO_STREAM(this->get_logger(), "Got calibratedGyroscopeData!");
					XsVector gyro = packet.calibratedGyroscopeData();

					imuMsgs_[device_num].angular_velocity.x = gyro[0];
					imuMsgs_[device_num].angular_velocity.y = gyro[1];
					imuMsgs_[device_num].angular_velocity.z = gyro[2];

					dotMsgs_[device_num].angular_velocity.x = gyro[0];
					dotMsgs_[device_num].angular_velocity.y = gyro[1];
					dotMsgs_[device_num].angular_velocity.z = gyro[2];
				}
				if (packet.containsCalibratedMagneticField())	// Filling in magnetometer data
				{
					RCLCPP_INFO_STREAM(this->get_logger(), "Got calibratedMagneticField!");
					XsVector mag = packet.calibratedMagneticField();

					magMsgs_[device_num].magnetic_field.x = mag[0];
					magMsgs_[device_num].magnetic_field.y = mag[1];
					magMsgs_[device_num].magnetic_field.z = mag[2];

					dotMsgs_[device_num].magnetic_field.x = mag[0];
					dotMsgs_[device_num].magnetic_field.y = mag[1];
					dotMsgs_[device_num].magnetic_field.z = mag[2];
				}

				// Triggers publishers
				RCLCPP_INFO_STREAM(this->get_logger(), "Preparing imu message...");
				imuMsgs_[device_num].header.stamp = this->get_clock()->now();
				RCLCPP_INFO_STREAM(this->get_logger(), "Getting device bluetooth ID...");
				imuMsgs_[device_num].header.frame_id = device->bluetoothAddress().toStdString();
				RCLCPP_INFO_STREAM(this->get_logger(), "Publishing IMU message...");
				imuPub_[device_num]->publish(imuMsgs_[device_num]);


				RCLCPP_INFO_STREAM(this->get_logger(), "Publishing magnetic field message...");
				magMsgs_[device_num].header.stamp = this->get_clock()->now();
				magMsgs_[device_num].header.frame_id = device->bluetoothAddress().toStdString();
				magPub_[device_num]->publish(magMsgs_[device_num]);

				RCLCPP_INFO_STREAM(this->get_logger(), "Publishing dotSensor message...");
				dotMsgs_[device_num].header.stamp = this->get_clock()->now();
				dotMsgs_[device_num].header.frame_id = device->bluetoothAddress().toStdString();
				dotPub_[device_num]->publish(dotMsgs_[device_num]);
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

	rclcpp::spin(std::make_shared<movella_dot::MovellaDot>("movella_dot"));
	rclcpp::shutdown();
	return 0;
}
