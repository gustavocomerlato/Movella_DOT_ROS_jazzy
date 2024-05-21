#include <iostream>
#include <iomanip>

#include "xdpchandler.h"

//ROS
#include "ros/ros.h"
#include "/home/reason1/movella_ws/devel/include/movella_dot/DotSensorMsg.h"
#include <string>
#include <vector>
#include <sstream>

using namespace std;

//--------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    ros::init(argc, argv,"Dot_ros");
    ros::NodeHandle n;

    ros::Publisher dotsensor_pub = n.advertise<movella_dot::DotSensorMsg>("DOT_Pos", 1000);
	ros::Rate loop_rate(10);

	XdpcHandler xdpcHandler;

	if (!xdpcHandler.initialize())
		return -1;

	xdpcHandler.scanForDots();

	if (xdpcHandler.detectedDots().empty())
	{
		cout << "No Movella DOT device(s) found. Aborting." << endl;
		xdpcHandler.cleanup();
		return -1;
	}

	xdpcHandler.connectDots();

	if (xdpcHandler.connectedDots().empty())
	{
		cout << "Could not connect to any Movella DOT device(s). Aborting." << endl;
		xdpcHandler.cleanup();
		return -1;
	}

	for (auto& device : xdpcHandler.connectedDots())
	{		
		auto filterProfiles = device->getAvailableFilterProfiles();
		cout << filterProfiles.size() << " available filter profiles:" << endl;
		for (auto& f : filterProfiles)
			cout << f.label() << endl;

		cout << "Current profile: " << device->onboardFilterProfile().label() << endl;
		if (device->setOnboardFilterProfile(XsString("General")))
			cout << "Successfully set profile to General" << endl;
		else
			cout << "Setting filter profile failed!" << endl;

		cout << "Putting device into measurement mode." << endl;
		if (!device->startMeasurement(XsPayloadMode::ExtendedEuler))
		{
			cout << "Could not put device into measurement mode. Reason: " << device->lastResultText() << endl;
			continue;
		}
	}

	cout << "\nMain loop. Publishing data to Ros topic" << endl;
	cout << string(83, '-') << endl;

	// First printing some headers so we see which data belongs to which device
	for (auto const& device : xdpcHandler.connectedDots())
		cout << setw(42) << left << device->bluetoothAddress();
	cout << endl;

	movella_dot::DotSensorMsg dot_msg;

	bool orientationResetDone = false;
	int64_t startTime = XsTime::timeStampNow();
	while (ros::ok())
	{
		if (xdpcHandler.packetsAvailable())
		{
			cout << "\r";
			for (auto const& device : xdpcHandler.connectedDots())
			{
				// Retrieve a packet
				XsDataPacket packet = xdpcHandler.getNextPacket(device->bluetoothAddress());

				std::string segment, seglist;
				std::stringstream test(device->bluetoothAddress().c_str());
				while(std::getline(test, segment, ':'))
				{
					seglist = segment;
				}

				if (packet.containsOrientation())
				{
					XsEuler euler = packet.orientationEuler();
					
					dot_msg.header.stamp = ros::Time::now();
					if (seglist == "56")
					{	
						cout <<"Dot_sens1";
						dot_msg.Dot_Sens1.x = euler.roll();
						dot_msg.Dot_Sens1.y = euler.pitch();
						dot_msg.Dot_Sens1.z = euler.yaw();
					}
					if (seglist == "6C")
					{
						cout <<"Dot_sens2";
						dot_msg.Dot_Sens2.x = euler.roll();
						dot_msg.Dot_Sens2.y = euler.pitch();
						dot_msg.Dot_Sens2.z = euler.yaw();
					}
					if (seglist == "7C")
					{
						cout <<"Dot_sens3";
						dot_msg.Dot_Sens3.x = euler.roll();
						dot_msg.Dot_Sens3.y = euler.pitch();
						dot_msg.Dot_Sens3.z = euler.yaw();
					}
					if (seglist == "3D")
					{
						cout <<"Dot_sens4";
						dot_msg.Dot_Sens4.x = euler.roll();
						dot_msg.Dot_Sens4.y = euler.pitch();
						dot_msg.Dot_Sens4.z = euler.yaw();
					}
					if (seglist == "69")
					{
						cout <<"Dot_sens5";
						dot_msg.Dot_Sens5.x = euler.roll();
						dot_msg.Dot_Sens5.y = euler.pitch();
						dot_msg.Dot_Sens5.z = euler.yaw();
					}
					dotsensor_pub.publish(dot_msg);
				}
			}
			cout << flush;
		}
		XsTime::msleep(0);
	}
	cout << "\n" << string(83, '-') << "\n";
	cout << endl;

	sleep(3);

	for (auto const& device : xdpcHandler.connectedDots())
	{
		cout << endl << "Resetting heading to default for device " << device->bluetoothAddress() << ": ";
		if (device->resetOrientation(XRM_DefaultAlignment))
			cout << "OK";
		else
			cout << "NOK: " << device->lastResultText();
	}
	cout << endl << endl;

	cout << "Stopping measurement..." << endl;
	for (auto device : xdpcHandler.connectedDots())
	{
		if (!device->stopMeasurement())
			cout << "Failed to stop measurement.";

		if (!device->disableLogging())
			cout << "Failed to disable logging.";
	}

	xdpcHandler.cleanup();
	ros::shutdown();
	return 0;
}
