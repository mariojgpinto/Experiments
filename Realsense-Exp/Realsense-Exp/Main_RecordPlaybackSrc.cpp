#include "Main_RecordPlaybackSrc.h"
// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.
#include <fstream>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <thread>

#include <time.h>

#include <chrono>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering

using namespace std::chrono;

const std::string no_camera_message = "No camera connected, please connect 1 or more";
const std::string platform_camera_name = "Platform Camera";
const std::string config_failure_message = std::string("Configuration file is somehow incorrect.\n") +
"\n" +
"Format:\n" +
"\n" +
"Execution Mode (\"record\" or \"playback\"" +
"Filename 1\n" +
"Filename 2\n" +
"Resolution in X Axis\n" +
"Resolution in Y Axis\n" +
"Framerate\n" +
"\n" +
"Files are typically in \".bag\" format.\n" +
"Aspect ratio must be 16:9. 4:3 is yet untested. Please check your Intel RealSense camera for possible resolutions.\n" +
"Resolutions including and beyond 720p require a frame rate of 30 or lower.\n" +
"\n" +
"CURRENTLY WORKING FOR 2 CAMERAS, AND NO MORE";

// Configuration vars
enum ExecutionMode { RECORD, PLAYBACK };
ExecutionMode executionMode;
int resolutionX;
int resolutionY;
int frameRate;
std::string file0;
std::string file1;
double delay0;
double delay1;

int streamID = 0;

const auto programStart = std::chrono::high_resolution_clock::now();
auto lastStep = std::chrono::high_resolution_clock::now();

bool readConfigurationFile() {
	std::ifstream configFile("recordconfig.cfg");
	if (configFile.is_open()) {
		std::string tempString;
		std::getline(configFile, tempString);
		if (tempString == "record")
			executionMode = RECORD;
		else if (tempString == "playback")
			executionMode = PLAYBACK;

		std::getline(configFile, file0);
		std::getline(configFile, file1);

		if (executionMode == RECORD
			&& (file0.substr(file0.size() - 4, 4) != ".bag" || file0.substr(file0.size() - 4, 4) != ".bag"))
			return false;

		std::getline(configFile, tempString);
		resolutionX = std::stoi(tempString);
		std::getline(configFile, tempString);
		resolutionY = std::stoi(tempString);

		if (executionMode == RECORD && (float)resolutionX / resolutionY != 16.f / 9.f)
			return false;

		std::getline(configFile, tempString);
		frameRate = std::stoi(tempString);

		if (executionMode == PLAYBACK) {
			std::getline(configFile, tempString);
			delay0 = std::stod(tempString);
			std::getline(configFile, tempString);
			delay1 = std::stod(tempString);
		}

		return true;
	}

	return false;
}

class device_container
{
	// Helper struct per pipeline
	struct view_port
	{
		std::map<int, rs2::frame> frames_per_stream;
		rs2::colorizer colorize_frame;
		texture tex;
		rs2::config config;
		rs2::pipeline pipe;
		rs2::pipeline_profile profile;
		double startDelay;
	};

public:

	void enable_device(rs2::device dev)
	{
		std::string serial_number(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
		std::lock_guard<std::mutex> lock(_mutex);

		if (_devices.find(serial_number) != _devices.end())
		{
			return; //already in
		}

		// Ignoring platform cameras (webcams, etc..)
		if (platform_camera_name == dev.get_info(RS2_CAMERA_INFO_NAME))
		{
			return;
		}
		// Create a pipeline from the given device
		rs2::pipeline p;
		rs2::config c;
		c.disable_all_streams();
		c.enable_stream(RS2_STREAM_DEPTH, -1, resolutionX, resolutionY, RS2_FORMAT_ANY, frameRate);
		if (!streamID)
			c.enable_record_to_file(file0);
		else
			c.enable_record_to_file(file1);
		c.enable_device(serial_number);

		// Start the pipeline with the configuration
		rs2::pipeline_profile profile = p.start(c);
		std::chrono::duration<double, std::milli> time = std::chrono::high_resolution_clock::now() - programStart;
		// Hold it internally
		_devices.emplace(serial_number, view_port{ {},{},{}, c, p, profile, time.count() });

		streamID++;
	}

	void enable_file_devices(int sID) {
		rs2::pipeline p;
		rs2::config c;
		if (!sID)
			c.enable_device_from_file(file0);
		else
			c.enable_device_from_file(file1);

		rs2::pipeline_profile profile;

		//if (sID) {
		//	std::this_thread::sleep_for(std::chrono::milliseconds((long)abs(delay0 - delay1)));
		//}

		profile = p.start(c);

		_devices.emplace(std::to_string(streamID), view_port{ {},{},{}, c, p, profile,{} });

		streamID++;
	}

	void remove_devices(const rs2::event_information& info)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		// Go over the list of devices and check if it was disconnected
		auto itr = _devices.begin();
		while (itr != _devices.end())
		{
			if (info.was_removed(itr->second.profile.get_device()))
			{
				itr = _devices.erase(itr);
			}
			else
			{
				++itr;
			}
		}
	}

	size_t device_count()
	{
		std::lock_guard<std::mutex> lock(_mutex);
		return _devices.size();
	}

	int stream_count()
	{
		std::lock_guard<std::mutex> lock(_mutex);
		int count = 0;
		for (auto&& sn_to_dev : _devices)
		{
			for (auto&& stream : sn_to_dev.second.frames_per_stream)
			{
				if (stream.second)
				{
					count++;
				}
			}
		}
		return count;
	}

	void poll_frames()
	{
		std::lock_guard<std::mutex> lock(_mutex);
		// Go over all device
		for (auto&& view : _devices)
		{
			// Ask each pipeline if there are new frames available
			rs2::frameset frameset;
			if (view.second.pipe.poll_for_frames(&frameset)) {
				for (int i = 0; i < frameset.size(); i++) {
					rs2::frame new_frame = frameset[i];
					int stream_id = new_frame.get_profile().unique_id();
					view.second.frames_per_stream[stream_id] = view.second.colorize_frame(new_frame); //update view port with the new stream
				}
			}
		}
	}

	void render_textures(int cols, int rows, float view_width, float view_height)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		int stream_no = 0;
		for (auto&& view : _devices) {
			// For each device get its frames
			for (auto&& id_to_frame : view.second.frames_per_stream) {
				// If the frame is available
				if (id_to_frame.second) {
					view.second.tex.upload(id_to_frame.second);
				}
				rect frame_location{
					view_width * (stream_no % cols),
					view_height * (stream_no / cols),
					view_width,
					view_height };
				if (rs2::video_frame vid_frame = id_to_frame.second.as<rs2::video_frame>()) {
					rect adjuested = frame_location.adjust_ratio({ static_cast<float>(vid_frame.get_width())
						, static_cast<float>(vid_frame.get_height()) });
					view.second.tex.show(adjuested);
					stream_no++;
				}
			}
		}
	}

	void writeConfigFile() {

		//record delays when recording
		std::ofstream configFileOutput;
		configFileOutput.open("recordconfig.cfg", std::fstream::out);

		configFileOutput << "playback" << std::endl; // Ensures that you can immediatelly play the recording after first execution
		configFileOutput << file0 << std::endl;
		configFileOutput << file1 << std::endl;
		configFileOutput << resolutionX << std::endl;
		configFileOutput << resolutionY << std::endl;
		configFileOutput << frameRate << std::endl;
		for (auto&& view : _devices) {
			configFileOutput << std::to_string(view.second.startDelay) << std::endl;
		}
	}

private:
	std::mutex _mutex;
	std::map<std::string, view_port> _devices;
};

int main_RecordPlaybackSrc(int argc, char* argv[])  try
{
	bool configFileSuccess = readConfigurationFile();

	// Create a simple OpenGL window for rendering:
	window app(1280, 960, "PROZIS - Depth Map Recorder");

	device_container connected_devices;

	rs2::context ctx;

	if (configFileSuccess) {
		// Register callback for tracking which devices are currently connected
		ctx.set_devices_changed_callback([&](rs2::event_information& info)
		{
			connected_devices.remove_devices(info);
			for (auto&& dev : info.get_new_devices())
			{
				connected_devices.enable_device(dev);
			}
		});

		if (executionMode == RECORD) {
			// Initial population of the device list
			for (auto&& dev : ctx.query_devices()) // Query the list of connected RealSense devices
			{
				connected_devices.enable_device(dev);
			}
		}
		else {
#pragma omp parallel for
			for (int i = 0; i < 2; i++) {
				connected_devices.enable_file_devices(i);
			}
		}
	}

	if (executionMode == RECORD)
		connected_devices.writeConfigFile();

	while (app)
	{
		if (!configFileSuccess) {
			draw_text(10, 10, config_failure_message.c_str());
			continue;
		}

		connected_devices.poll_frames();

		int total_number_of_streams;
		if (executionMode == RECORD)
			total_number_of_streams = connected_devices.stream_count();
		else
			total_number_of_streams = 2;

		if (total_number_of_streams == 0)
		{
			draw_text(int(std::max(0.f, (app.width() / 2) - no_camera_message.length() * 3)),
				int(app.height() / 2), no_camera_message.c_str());
			continue;
		}
		if (connected_devices.device_count() == 1)
		{
			draw_text(0, 10, "Please connect another camera");
		}
		int cols = int(std::ceil(std::sqrt(total_number_of_streams)));
		int rows = int(std::ceil(total_number_of_streams / static_cast<float>(cols)));

		float view_width = (app.width() / cols);
		float view_height = (app.height() / rows);

		connected_devices.render_textures(cols, rows, view_width, view_height);
	}

	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr <<
		"RealSense error calling " <<
		e.get_failed_function() <<
		"(" <<
		e.get_failed_args() <<
		"):\n    " <<
		e.what() <<
		std::endl;

	getchar();
	return EXIT_FAILURE;
}
catch (const std::exception& e)
{
	std::cerr << e.what() << std::endl;
	getchar();
	return EXIT_FAILURE;
}