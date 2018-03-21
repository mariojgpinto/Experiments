#include "SharedFunctions.h"


std::string depthWindowNameRealsense[] = { "Depth0", "Depht1", "Depht2", "Depht3" };
std::string maskWindowNameRealsense[] = { "Mask0", "Mask1", "Mask2", "Mask3" };
std::string colorWindowNameRealsense[] = { "Color0", "Color1", "Color2", "Color3" };

void CreateWindows_RealsenseKinects(int nKinects) {
	int windowDeltaY = -1200;
	int windowDeltaX = 960;
	int windowSpaceWidth = 640;
	int windowSpaceHeight = 480;

	for (int i = 0; i < nKinects; ++i) {
		cv::namedWindow(depthWindowNameRealsense[i]);
		cv::moveWindow(depthWindowNameRealsense[i], windowDeltaX + windowSpaceWidth * i, 0 + windowDeltaY);

		cv::namedWindow(maskWindowNameRealsense[i]);
		cv::moveWindow(maskWindowNameRealsense[i], windowDeltaX + windowSpaceWidth * i, windowSpaceHeight + windowDeltaY);

		cv::namedWindow(colorWindowNameRealsense[i]);
		cv::moveWindow(colorWindowNameRealsense[i], windowSpaceWidth * i, 0);
	}
}


static std::string get_device_name(const rs2::device& dev)
{
	// Each device provides some information on itself, such as name:
	std::string name = "Unknown Device";
	if (dev.supports(RS2_CAMERA_INFO_NAME))
		name = dev.get_info(RS2_CAMERA_INFO_NAME);

	// and the serial number of the device:
	std::string sn = "########";
	if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
		sn = std::string("#") + dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

	return name + " " + sn;
}

rs2::device get_a_realsense_device()
{
	// First, create a rs2::context.
	// The context represents the current platform with respect to connected devices
	rs2::context ctx;

	// Using the context we can get all connected devices in a device list
	rs2::device_list devices = ctx.query_devices();

	rs2::device selected_device;
	if (devices.size() == 0)
	{
		std::cerr << "No device connected, please connect a RealSense device" << std::endl;

		//To help with the boilerplate code of waiting for a device to connect
		//The SDK provides the rs2::device_hub class
		rs2::device_hub device_hub(ctx);

		//Using the device_hub we can block the program until a device connects
		selected_device = device_hub.wait_for_device();
	}
	else
	{
		std::cout << "Found the following devices:\n" << std::endl;

		// device_list is a "lazy" container of devices which allows
		//The device list provides 2 ways of iterating it
		//The first way is using an iterator (in this case hidden in the Range-based for loop)
		int index = 0;
		for (rs2::device device : devices)
		{
			std::cout << "  " << index++ << " : " << get_device_name(device) << std::endl;
		}


		// Update the selected device
		selected_device = devices[0];
	}

	return selected_device;
}

float get_depth_scale(rs2::device dev)
{
	// Go over the device's sensors
	for (rs2::sensor& sensor : dev.query_sensors())
	{
		// Check if the sensor if a depth sensor
		if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
		{
			return dpt.get_depth_scale();
		}
	}
	throw std::runtime_error("Device does not have a depth sensor");
}


