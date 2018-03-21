#include "Main_RecordPlaybackCV.h"
#include "SharedFunctions.h"

using namespace cv;
using namespace std;

int main_RecordPlaybackCV(int argc, char* argv[]) try {
	printf("main_RecordPlaybackCV\n");

	cv::setUseOptimized(true);

	double _last_tick = 0;
	int _frame_counter = 0;
	float _frame_rate = 0;

	// Declare depth colorizer for pretty visualization of depth data
	//rs2::colorizer color_map;
	//rs2::
	//rs2::config cfg;
	//cfg.enable_stream(RS2_STREAM_INFRARED, 1);
	//cfg.enable_stream(RS2_STREAM_INFRARED, 2);
	//rs2::pipeline pipe;
	//pipe.start(cfg);

	// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;
	// We want the points object to be persistent so we can display the last cloud when a frame drops
	rs2::points points;


	rs2::context* ctx = new rs2::context();
	rs2::config* conf = new rs2::config();

	conf->disable_all_streams();
	//conf->enable_stream(RS2_STREAM_COLOR, 0, 640, 480, RS2_FORMAT_RGB8, 30);
	conf->enable_stream(RS2_STREAM_DEPTH, -1, 640, 360, RS2_FORMAT_ANY, 30);
	//conf->enable_record_to_file("record.bag");

	//conf->
	//rs2::options option();
	//rs2_set_option(option, rs2_option::RS2_OPTION_ACCURACY, 1,0)
	// = new rs2::options(rs2_option::RS2_OPTION_ACCURACY);
	//option.set_option(rs2_option::RS2_OPTION_ACCURACY, 1);
	//rs2::
	//rs2::rs2_load_json()




	rs2::pipeline *pipe = new rs2::pipeline(*ctx);
	/*rs2::pipeline_profile* profile = &*/pipe->start(*conf);
	//rs2::device device = get_a_realsense_device();

	//rs2_error* e = nullptr;
	//rs2_create_record_device(device.get().get(), "my_file_name.bag", &e);
	//


	//if (device.is<rs400::advanced_mode>())
	//{
	//	auto advanced_mode_dev = device.as<rs400::advanced_mode>();
	//	// Check if advanced-mode is enabled
	//	if (!advanced_mode_dev.is_enabled())
	//	{
	//		// Enable advanced-mode
	//		advanced_mode_dev.toggle_advanced_mode(true);
	//	}

	//	//std::ifstream t("C:\\Dev\\Tech\\Intel\\Conf_HighAccuracy.json");
	//	std::ifstream t("C:\\Dev\\Tech\\Intel\\Conf_HighDensity.json");
	//	std::string str((std::istreambuf_iterator<char>(t)),
	//		std::istreambuf_iterator<char>());
	//	advanced_mode_dev.load_json(str);
	//}
	//else
	//{
	//	std::cout << "Current device doesn't support advanced-mode!\n";
	//	return EXIT_FAILURE;
	//}
	
	cv::namedWindow("Depth Mat");

	const int max_slider_max = 5000;
	const int min_slider_max = 5000;
	int max_slider = 2000;
	int min_slider = 500;
	cv::createTrackbar("Min", "Depth Mat", &min_slider, min_slider_max);
	cv::createTrackbar("Max", "Depth Mat", &max_slider, max_slider_max);

	rs2::recorder* recorder = NULL;

	char c = '2';
	while (c != 27)
	{
		rs2::frameset data;
		bool result = pipe->poll_for_frames(&data);// Wait for next set of frames from the camera

		if (result) {
			rs2::depth_frame depth = data.get_depth_frame();
			//rs2::video_frame color = data.get_color_frame();

			//points = pc.calculate(depth);

			Mat depthMatRaw(Size(depth.get_width(), depth.get_height()), CV_16UC1, (void*)depth.get_data());

			cv::Mat mask;
			cv::Mat depthMat_range(depth.get_height(), depth.get_width(), CV_8UC1);
			cv::inRange(depthMatRaw, min_slider, max_slider, depthMat_range);


			double min;
			double max;
			cv::minMaxIdx(depthMatRaw, &min, &max);
			cv::Mat depthMat_test;
			cv::Mat depthMat;
			// Histogram Equalization
			float scale = 255 / (5000.0f);
			depthMatRaw.convertTo(depthMat_test, CV_8U, -255.0f / 5000.0f, 255.0f);
			depthMat_test.copyTo(depthMat, depthMat_range);

			imshow("Depth Mat", depthMat);
		}

		c = waitKey(5);
		if (c == ' ') {
			printf("Space\n");

			auto devices = ctx->query_devices();
			if (devices.size() > 0)
			{
				//profile->get_device().
				//conf->enable_record_to_file("record.bag");
				//pipe->stop();
				//pipe->start(*conf);
				
				//pipe->get_active_profile().get_device().s

				//sn_to_dev.second.config.enable_record_to_file(file0);

				//sn_to_dev.second.pipe.stop();
				//sn_to_dev.second.profile = sn_to_dev.second.pipe.start(sn_to_dev.second.config);

				
				recorder = new rs2::recorder("mfile.bag", pipe->get_active_profile().get_device());
				//recorder->pause();
				//recorder->resume();
				//recorder.

				//rs2_error* e = nullptr;
				//rs2_config_enable_record_to_file(conf->get().get(), "my_file_name.bag", &e);
				//rs2_create_record_device(device.get().get(), "my_file_name.bag", &e);
				printf("Start Recording");



				//rs2_create_record_device
				//Create a rs2::recorder from the first device, and desired file name
				//'.bag' is the common extension for rosbag files
				//printf("rs2::recorder created\n");
				//rs2::recorder device("my_file_name.bag", pipe->get_active_profile().get_device());
				//rs2_record_device_filename
				//device.st
				//device.();
				//recorder "is a" device, so just use it like any other device now
			}

			//device

			//pipe->stop();
			//conf->enable_record_to_file("file.bag");
			//pipe->start(*conf);

		}

		++_frame_counter;
		if (_frame_counter == 30)
		{
			double current_tick = cv::getTickCount();
			_frame_rate = _frame_counter / ((current_tick - _last_tick) / cv::getTickFrequency());
			_last_tick = current_tick;
			_frame_counter = 0;
			printf("FrameRate %.2f\n", _frame_rate);
		}
	}

	pipe->stop();

	if (recorder != NULL) {
		delete recorder;
	}

	delete conf;
	delete pipe;
	//delete profile;
	delete ctx;

	return 0;
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