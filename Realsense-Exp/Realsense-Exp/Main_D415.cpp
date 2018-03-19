#include "Main_D415.h"
#include "SharedFunctions.h"

int main_d415(int argc, char* argv[]) {
	printf("main_d415");

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

	//ctx->lo

	//rs2::device* dev
	conf->disable_all_streams();
	//conf->enable_stream(RS2_STREAM_COLOR, 0, 640, 480, RS2_FORMAT_RGB8, 30);
	conf->enable_stream(RS2_STREAM_DEPTH, 0, 640, 480, RS2_FORMAT_Z16, 90);

	//conf->
	//rs2::options option();
	//rs2_set_option(option, rs2_option::RS2_OPTION_ACCURACY, 1,0)

	///* = new rs2::options(rs2_option::RS2_OPTION_ACCURACY);*/
	//option.set_option(rs2_option::RS2_OPTION_ACCURACY, 1);

	//rs2::

	//rs2::rs2_load_json()

	rs2::pipeline* pipe = new rs2::pipeline(*ctx);
	pipe->start(*conf);
	rs2::device device = get_a_realsense_device();


	if (device.is<rs400::advanced_mode>())
	{
		auto advanced_mode_dev = device.as<rs400::advanced_mode>();
		// Check if advanced-mode is enabled
		if (!advanced_mode_dev.is_enabled())
		{
			// Enable advanced-mode
			advanced_mode_dev.toggle_advanced_mode(true);

		}

		//std::ifstream t("C:\\Dev\\Tech\\Intel\\Conf_HighAccuracy.json");
		std::ifstream t("C:\\Dev\\Tech\\Intel\\Conf_HighDensity.json");
		std::string str((std::istreambuf_iterator<char>(t)),
			std::istreambuf_iterator<char>());

		advanced_mode_dev.load_json(str);
	}
	else
	{
		std::cout << "Current device doesn't support advanced-mode!\n";
		return EXIT_FAILURE;
	}

	//advac

	//rs2::e
	//rs2_load_json(&device, 

	//device.e
	//rs2::device::


	//ctx->




	// Declare RealSense pipeline, encapsulating the actual device and sensors
	//rs2::pipeline pipe;
	// Start streaming with default recommended configuration
	//rs2::pipeline_profile profile = pipe.start();



	//
	//const auto window_name = "Display Image";
	//namedWindow(window_name, WINDOW_AUTOSIZE);

	cv::namedWindow("Depth Mat");

	const int max_slider_max = 5000;
	const int min_slider_max = 5000;
	int max_slider = 2000;
	int min_slider = 500;
	cv::createTrackbar("Min", "Depth Mat", &min_slider, min_slider_max);
	cv::createTrackbar("Max", "Depth Mat", &max_slider, max_slider_max);

	char c = '2';
	while (c != 27)
	{

		//pipe->
		rs2::frameset data = pipe->wait_for_frames(); // Wait for next set of frames from the camera
		rs2::depth_frame depth = data.get_depth_frame();
		//rs2::video_frame color = data.get_color_frame();

		points = pc.calculate(depth);

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

		//Mat colorMat(Size(color.get_width(), color.get_height()), CV_8UC3, (void*)color.get_data());
		//cvtColor(colorMat, colorMat, COLOR_RGB2BGR); //dont forget that imshow expect BGR image



		//float depth_scale = get_depth_scale(profile.get_device());

		//printf("depth_scale: %f\n", depth_scale);

		int x1 = depth.get_width() / 3; int x2 = x1 * 2; int y = depth.get_height() / 2;
		int coord1 = depth.get_width() * y + x1;
		int coord2 = depth.get_width() * y + x2;

		auto vertices = points.get_vertices();

		rs2::vertex point1 = vertices[coord1];
		rs2::vertex point2 = vertices[coord2];

		double myDistance = sqrt(pow(point1.x - point2.x, 2.0) + pow(point1.y - point2.y, 2.0) + pow(point1.z - point2.z, 2.0));

		cv::circle(depthMat, cv::Point(x1, y), 7, cv::Scalar(255), -1);
		cv::circle(depthMat, cv::Point(x1, y), 5, cv::Scalar(0), -1);
		cv::circle(depthMat, cv::Point(x1, y), 3, cv::Scalar(255), -1);


		cv::circle(depthMat, cv::Point(x2, y), 7, cv::Scalar(255), -1);
		cv::circle(depthMat, cv::Point(x2, y), 5, cv::Scalar(0), -1);
		cv::circle(depthMat, cv::Point(x2, y), 3, cv::Scalar(255), -1);

		//printf("Distance: %f\n", myDistance);
		//printf("Middle(%.2f,%.2f,%.2f) %d \n", point.x, point.y, point.z, points.size());


		//imshow("Color Mat", colorMat);
		imshow("Depth Mat", depthMat);

		c = waitKey(5);
		if (c == ' ') {
			printf("Space\n");

			auto devices = ctx->query_devices();
			if (devices.size() > 0)
			{

				//std::shared_ptr<rs2_device> _dev;
				//rs2_error* e = nullptr;
				//_dev = std::shared_ptr<rs2_device>(
				//	rs2_create_record_device(device.get().get(), "my_file_name.bag", &e),
				//	rs2_delete_device);

				//rs2_re


				printf("");
				//rs2_create_record_device
				//Create a rs2::recorder from the first device, and desired file name
				//'.bag' is the common extension for rosbag files
				//printf("rs2::recorder created\n");
				//rs2::recorder device("my_file_name.bag", devices[0]);
				//device.();
				//recorder "is a" device, so just use it like any other device now
			}

			//device

			//pipe->stop();
			//conf->enable_record_to_file("file.bag");
			//pipe->start(*conf);

		}

		++_frame_counter;
		if (_frame_counter == 15)
		{
			double current_tick = cv::getTickCount();
			_frame_rate = _frame_counter / ((current_tick - _last_tick) / cv::getTickFrequency());
			_last_tick = current_tick;
			_frame_counter = 0;
			printf("FrameRate %.2f\n", _frame_rate);
		}
	}

	return 0;
}