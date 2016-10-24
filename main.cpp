#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <random>
#include <algorithm>

#include "tank_detection.h"
#include "components.h"
#include "motion.h"

int main(int argc, char** argv) {

	if (argc != 2) {
		std::cout << "Please pass a filename for the movie" << std::endl;
		exit(1);
	}

	std::vector<cv::Vec3b> color_vector;
	color_vector.resize(300);

	std::mt19937 rng;
	rng.seed(std::random_device()());
	std::uniform_int_distribution<std::mt19937::result_type> dist(0, 255);

	color_vector[0] = cv::Vec3b(0, 0, 0);
	for (ushort l = 1; l < color_vector.size(); ++l) {
		color_vector[l] = cv::Vec3b(dist(rng), dist(rng), dist(rng));
	}

	const std::string source = argv[1];
	cv::VideoCapture video_capture(source);
	if (!video_capture.isOpened()){
		std::cout << "Could not open the input video: " << source << std::endl;
		exit(1);
	}

	cv::Mat source_image;
	cv::Mat hsv_image;

	video_capture >> source_image;
	cv::cvtColor(source_image, hsv_image, cv::COLOR_BGR2HSV);

	cv::Rect2i left_bounds, right_bounds;
	detect_tank(hsv_image, left_bounds, right_bounds);

	cv::Mat left_image;
	cv::Mat left_filtered_image;
	cv::Mat left_hsv_image(left_bounds.size(), CV_8UC3);
	cv::Mat left_hsv_channels[3];
	cv::Mat previous_left_hsv_channels[] {
		cv::Mat::zeros(left_bounds.size(), CV_8U),
		cv::Mat::zeros(left_bounds.size(), CV_8U),
		cv::Mat::zeros(left_bounds.size(), CV_8U)
	};
	cv::Mat previous_left_v_channel;
	cv::Mat left_v_difference;
	cv::Mat left_threshold_image(left_bounds.size(), CV_8UC1);
	cv::Mat left_label_image(left_bounds.size(), CV_16U);
	cv::Mat left_segmented_image(left_bounds.size(), CV_8UC3);
	cv::Mat left_energy_image = cv::Mat::zeros(left_bounds.size(), CV_8UC1);
	cv::Mat left_action_image = cv::Mat::zeros(left_bounds.size(), CV_8UC1);
	cv::Mat left_search_mask;

	cv::Mat right_image;

	cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));

	for (uint frame_index = 0;; ++frame_index){
		video_capture >> source_image;
		if (source_image.empty()) break;

		left_image = source_image(left_bounds).clone();

		cv::GaussianBlur(left_image, left_filtered_image, cv::Size(0, 0), 3);
		cv::addWeighted(left_image, 1.5, left_filtered_image, -0.5, 0, left_filtered_image);
		cv::bilateralFilter(left_filtered_image.clone(), left_filtered_image, 11, 75, 75);

		cv::cvtColor(left_filtered_image, left_hsv_image, cv::COLOR_BGR2HSV);

		if (frame_index > 0) {
			previous_left_v_channel = left_hsv_channels[2].clone();

		}

		cv::split(left_hsv_image, left_hsv_channels);

		cv::adaptiveThreshold(left_hsv_channels[2], left_threshold_image, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 31, 7);

		left_v_difference = left_hsv_channels[2] - previous_left_v_channel;
		cv::threshold(left_v_difference, left_v_difference, 3, 255, CV_THRESH_BINARY);

		energy(left_v_difference, left_energy_image, std::min(16, static_cast<int>(frame_index >> 1)));
		cv::threshold(left_energy_image, left_action_image, 1, 255, CV_THRESH_BINARY);
		cv::morphologyEx(left_action_image, left_action_image, cv::MORPH_CLOSE, element);
		cv::morphologyEx(left_action_image, left_action_image, cv::MORPH_OPEN, element);

		cv::bitwise_and(left_action_image, left_threshold_image, left_search_mask);

		cv::connectedComponents(left_search_mask, left_label_image, 8, CV_16U);
		left_segmented_image = 0;
		colorize_components(left_label_image, color_vector, left_segmented_image);

		right_image = source_image(right_bounds).clone();

		cv::imshow("left_image", left_image);
		cv::imshow("left_threshold_image", left_threshold_image);
		cv::imshow("left_segmented_image", left_segmented_image);
		cv::imshow("left_energy_image", left_energy_image);
		cv::imshow("left_action_image", left_action_image);
		cv::imshow("left_search_mask", left_search_mask);
		cv::imshow("left_hsv_image", left_hsv_image);

		if (cv::waitKey(30) >= 0) break;
	}
}

