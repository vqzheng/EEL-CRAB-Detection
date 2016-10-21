#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

#include "tank_detection.h"

int main(int argc, char** argv) {

	if (argc != 2) {
		std::cout << "Please pass a filename for the movie" << std::endl;
		exit(1);
	}

	const std::string source = argv[1];
	cv::VideoCapture video_capture(source);
	if (!video_capture.isOpened()){
		std::cout << "Could not open the input video: " << source << std::endl;
		exit(1);
	}

	cv::Mat frame;
	cv::Mat edge;
	cv::Ptr<cv::BackgroundSubtractor> pMOG2 = cv::createBackgroundSubtractorMOG2();

	video_capture >> frame;
	//cv::resize(frame, frame, cv::Size(512, 512));

	cv::Mat hsv_frame;
	cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

	cv::Rect2i left_bounds, right_bounds;
	detect_tank(hsv_frame, left_bounds, right_bounds);

	for (;;){
		video_capture >> frame;

		//cv::resize(frame, frame, cv::Size(512, 512));

		cv::Mat left_tank = frame(left_bounds).clone();
		//cv::resize(left_tank, left_tank, cv::Size(512, 512));

		cv::Mat right_tank = frame(right_bounds).clone();
		//cv::resize(right_tank, right_tank, cv::Size(512, 512));

		cvtColor(frame, edge, CV_BGR2GRAY);
		cv::GaussianBlur(edge, edge, cv::Size(3, 3), 1.5, 1.5);
		pMOG2->apply(edge, frame);
		cv::Canny(edge, edge, 0, 45, 3);

		cv::Mat display_image;
		cv::cvtColor(edge, display_image, cv::COLOR_GRAY2BGR);

		cv::rectangle(display_image, left_bounds, cv::Scalar(255, 0, 0), 4);
		cv::rectangle(display_image, right_bounds, cv::Scalar(0, 0, 255), 4);

		//cv::imshow("output", display_image);
		cv::imshow("left tank", left_tank);
		cv::imshow("right tank", right_tank);
		

		if (cv::waitKey(30) >= 0) break;
	}
}