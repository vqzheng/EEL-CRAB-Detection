#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/video/tracking.hpp>
#include <iostream>
#include <random>
#include <algorithm>

#include "tank_detection.h"
#include "components.h"
#include "motion.h"
#include "skeleton.h"
#include "object_analysis.h"

static void drawOptFlowMap(const cv::Mat& flow, cv::Mat& cflowmap, int step,
	double, const cv::Scalar& color);

void flow_mag_filter(const cv::Mat& flow, cv::Mat &mask, float mag);

/*
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
	cv::Mat left_keypoints_image;
	cv::Mat left_keypoints_mask;
	cv::Mat left_filtered_image;
	cv::Mat left_hsv_image(left_bounds.size(), CV_8UC3);
	cv::Mat left_hsv_channels[3];
	cv::Mat previous_left_v_channel;
	cv::Mat left_v_difference;
	cv::Mat left_threshold_image(left_bounds.size(), CV_8UC1);
	cv::Mat left_label_image(left_bounds.size(), CV_16U);
	cv::Mat left_segmented_image(left_bounds.size(), CV_8UC3);
	cv::Mat left_energy_image = cv::Mat::zeros(left_bounds.size(), CV_8UC1);
	cv::Mat left_action_image = cv::Mat::zeros(left_bounds.size(), CV_8UC1);
	cv::Mat left_search_mask;
	cv::Ptr<cv::BackgroundSubtractor> pMOG2 = cv::createBackgroundSubtractorMOG2();

	cv::Mat right_image;

	cv::Mat erosion_element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
	cv::Mat dilation_element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(13, 13));

	for (uint frame_index = 0;; ++frame_index){
		video_capture >> source_image;
		if (source_image.empty()) break;
		left_image = source_image(left_bounds).clone();

		// SHARPENING TEST
		//cv::Mat blurred;
		//cv::blur(left_image, blurred, cv::Size(30, 30));
		//cv::addWeighted(left_image.clone(), 4, blurred, -4, 128, left_image);

		cv::GaussianBlur(left_image, left_filtered_image, cv::Size(0, 0), 3);
		pMOG2->apply(left_filtered_image, left_keypoints_image);
		cv::threshold(left_keypoints_image, left_keypoints_mask, 10, 255, cv::THRESH_BINARY);

		cv::addWeighted(left_image, 1.5, left_filtered_image, -0.5, 0, left_filtered_image);
		cv::bilateralFilter(left_filtered_image.clone(), left_filtered_image, 11, 75, 75);
		
		left_filtered_image = left_image;

		cv::cvtColor(left_filtered_image, left_hsv_image, cv::COLOR_BGR2HSV);
	
		if (frame_index > 0) {
			previous_left_v_channel = left_hsv_channels[2].clone();
		}

		cv::split(left_hsv_image, left_hsv_channels);

		cv::adaptiveThreshold(left_hsv_channels[2], left_threshold_image, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 31, 7);
		//cv::threshold(left_hsv_channels[2], left_threshold_image, 108, 255, cv::THRESH_BINARY_INV);

		left_v_difference = left_hsv_channels[2] - previous_left_v_channel;
		cv::threshold(left_v_difference, left_v_difference, 20, 255, CV_THRESH_BINARY);

		energy(left_v_difference, left_energy_image, std::min(2, static_cast<int>(frame_index >> 1)));
		cv::threshold(left_energy_image, left_action_image, 1, 255, CV_THRESH_BINARY);
		cv::morphologyEx(left_action_image, left_action_image, cv::MORPH_ERODE, erosion_element);
		cv::morphologyEx(left_action_image, left_action_image, cv::MORPH_DILATE, dilation_element);

		cv::bitwise_and(left_keypoints_mask, left_threshold_image, left_search_mask);

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
		cv::imshow("left_keypoints_mask", left_keypoints_mask);

		if (cv::waitKey(10) >= 0) break;
	}
}
*/

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

	std::vector<cv::Point2i> last_centroids;

	int iLastX = -1;
	int iLastY = -1;

	cv::Mat left_image,
		left_flow,
		cleft_flow,
		left_tracked,
		left_label,
		type_mask(left_bounds.size(), CV_8UC1),
		left_cc_label,
		cleft_label,
		left_mag_mask(left_bounds.size(), CV_8UC1);
	cv::UMat uleft_image, 
		uleft_previous_image, 
		uleft_flow;
	cv::cvtColor(source_image(left_bounds).clone(), uleft_previous_image, cv::COLOR_BGR2GRAY);
	cv::Mat imgLines = cv::Mat::zeros(left_mag_mask.size(), CV_8UC3);

	cv::Mat right_tank;

	for (uint frame_index = 0;; ++frame_index){
		video_capture >> source_image;
		if (source_image.empty()) break;
		left_image = source_image(left_bounds).clone();
		right_tank = source_image(right_bounds).clone();

		cv::medianBlur(left_image, left_image, 7);

		cv::cvtColor(left_image, uleft_image, cv::COLOR_BGR2GRAY);

		cv::calcOpticalFlowFarneback(uleft_previous_image, uleft_image, uleft_flow, 0.7, 5, 15, 3, 5, 1.2, 0);
		uleft_previous_image = uleft_image.clone();
		uleft_flow.copyTo(left_flow); 
		
		left_mag_mask = 0;
		flow_mag_filter(left_flow, left_mag_mask, 0.5f);

		const cv::Mat close_element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
		const cv::Mat dilate_element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 11));
		//cv::morphologyEx(left_mag_mask, left_mag_mask, cv::MORPH_DILATE, dilate_element);
		cv::morphologyEx(left_mag_mask, left_mag_mask, cv::MORPH_CLOSE, close_element);
		//cv::morphologyEx(left_mag_mask, left_mag_mask, cv::MORPH_DILATE, dilate_element);
		cv::morphologyEx(left_mag_mask, left_mag_mask, cv::MORPH_OPEN, close_element);
		cv::morphologyEx(left_mag_mask, left_mag_mask, cv::MORPH_CLOSE, close_element);
		//cv::morphologyEx(left_mag_mask, left_mag_mask, cv::MORPH_DILATE, dilate_element);

		cleft_label = left_image.clone();
		int labels = cv::connectedComponents(left_mag_mask, left_label, 8, CV_16U);

		float mean = 0.0f;
		float stddev = 0.0f;

		for (int l = 1; l < labels; ++l) {
			left_cc_label = 0;
			left_cc_label = (left_label == l);
			cv::Moments m = cv::moments(left_cc_label, false);
			mean += m.m00;
		} mean /= labels;

		for (int l = 1; l < labels; ++l) {
			left_cc_label = 0;
			left_cc_label = (left_label == l);
			cv::Moments m = cv::moments(left_cc_label, false);
			const float u = m.m00 - mean;
			stddev += u * u;
		} stddev = sqrt(stddev / labels);

		cv::bitwise_and(left_cc_label, left_mag_mask, left_cc_label);
		type_mask = 0;
		for (int l = 1; l < labels; ++l) {
			left_cc_label = 0;
			left_cc_label = (left_label == l);
			cv::Moments m = cv::moments(left_cc_label, false);
			const float z = (m.m00 - mean) / stddev;
			if (z > 1.0f && m.m00 > 100000 ) {
				type_mask += left_cc_label;
				//std::cout << m.m00 << std::endl;
			}
		}

		cv::bitwise_and(type_mask, left_mag_mask, left_mag_mask);
		find_skeleton(left_mag_mask, left_mag_mask);
		cv::morphologyEx(left_mag_mask, left_mag_mask, cv::MORPH_DILATE, dilate_element);
		const cv::Mat dilate_element2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
		const cv::Mat open_element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
		cv::morphologyEx(left_mag_mask, left_mag_mask, cv::MORPH_DILATE, dilate_element);
		cv::morphologyEx(left_mag_mask, left_mag_mask, cv::MORPH_OPEN, open_element);
		cv::morphologyEx(left_mag_mask, left_mag_mask, cv::MORPH_ERODE, dilate_element);
		//cv::morphologyEx(left_mag_mask, left_mag_mask, cv::MORPH_ERODE, dilate_element);
		//cv::morphologyEx(left_mag_mask, left_mag_mask, cv::MORPH_DILATE, dilate_element);
		std::vector<cv::Point2i> centroid_vector;
		labels = cv::connectedComponents(left_mag_mask, left_label, 8, CV_16U);
		calculate_centroids(left_label, labels, centroid_vector);

		colorize_components(left_label, color_vector, cleft_label);

		//cv::Mat imgLines = cv::Mat::zeros(left_mag_mask.size(), CV_8UC3);
		std::cout << centroid_vector.size() << std::endl;
		for (int i = 1; i < centroid_vector.size();i++){
			//cv::circle(imgLines, centroid_vector[i], 5 , cv::Scalar(0, 0, 255));
			int n = 0;
			double current_min = 100000000000;
			for (int j = 1; j < last_centroids.size(); j++){
				double distance;
				distance = cv::norm(centroid_vector[i] - last_centroids[j]);
				if (distance < current_min && distance < 60){
					n = j;
				}
				if (n!=0)
				cv::line(imgLines, last_centroids[n], centroid_vector[i], cv::Scalar(0, 0, 255), 1, 8, 0);

			}
		}
		last_centroids = centroid_vector;

		left_tracked = 0;
		left_image.copyTo(left_tracked, left_mag_mask);

		cleft_flow = left_image.clone();
		drawOptFlowMap(left_flow, cleft_flow, 16, 1.5, cv::Scalar(0, 255, 0));

		//CRABS
		/*cv::Mat basic_right, crabs_threshold;
		cv::cvtColor(right_tank, basic_right, CV_BGR2HSV);
		//cv::imshow("crabs hsv", basic_right);
		cv::inRange(basic_right, cv::Scalar(0, 0, 224), cv::Scalar(179, 255, 255), crabs_threshold);
		cv::imshow("crabs thresholded", crabs_threshold);

		std::vector<std::vector<cv::Point>> contoursCrab;
		std::vector<cv::Vec4i> hierarchyCrab;
		cv::findContours(crabs_threshold, contoursCrab, hierarchyCrab, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
		if (contoursCrab.size() >= 1) {
			for (int i = 0; i < contoursCrab.size(); i++) {
				double area = contourArea(contoursCrab[i]);
				if (area > 45 && area < 60) {
					//cv::Scalar color = cv::Scalar(dist(rng), dist(rng), dist(rng));
					cv::drawContours(right_tank, contoursCrab, i, cv::Scalar(255, 0, 0), 2, 8, hierarchyCrab);
					cv::Moments u = cv::moments(contoursCrab[i], false);
					cv::Point2f center = cv::Point2f(u.m10 / u.m00, u.m01 / u.m00); //centroid
					//center is in this form: [234.566, 283.123];
					cv::circle(right_tank, center, 2, cv::Scalar(0, 0, 255), CV_FILLED);
				}
			}
			cv::imshow("contour crabs", right_tank);
		}*/





		cv::Moments oMoments = moments(left_mag_mask);

		double dM01 = oMoments.m01;
		double dM10 = oMoments.m10;
		double dArea = oMoments.m00;
		
		
		

		// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
		if (dArea > 0)
		{
			//calculate the position of the ball
			int posX = dM10 / dArea;
			int posY = dM01 / dArea;


			if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
			{
				//Draw a red line from the previous point to the current point
				//std::cout << posY << std::endl;
				//cv::circle(imgLines, cv::Point(posX, posY), 10, cv::Scalar(0, 0, 255));
				//cv::line(imgLines, cv::Point(posX, posY), cv::Point(iLastX, iLastY), cv::Scalar(0, 0, 255), 2);
			}
			//std::cout << posY << std::endl;

			iLastX = posX;
			iLastY = posY;
		}

		cv::imshow("tracking", imgLines);
		cv::imshow("uleft_image", uleft_image);
		cv::imshow("cleft_flow", cleft_flow);
		cv::imshow("left_tracked", left_tracked);
		cv::imshow("left_mag_mask", left_mag_mask);
		cv::imshow("cleft_label", cleft_label);

		if (cv::waitKey(30) == 'q') break;

	}

}

static void drawOptFlowMap(const cv::Mat& flow, cv::Mat& cflowmap, int step,
	double, const cv::Scalar& color)
{
	for (int y = 0; y < cflowmap.rows; y += step)
		for (int x = 0; x < cflowmap.cols; x += step)
		{
			const cv::Point2f& fxy = flow.at<cv::Point2f>(y, x);
			line(cflowmap, cv::Point(x, y), cv::Point(cvRound(x + fxy.x), cvRound(y + fxy.y)),
				color);
			circle(cflowmap, cv::Point(x, y), 2, color, -1);
		}
}

void flow_mag_filter(const cv::Mat& flow, cv::Mat &mask, float mag) {
	const float mag_sqr = mag * mag;

	for (int r = 0; r < flow.rows; ++r) {
		const cv::Point2f *flow_ptr = flow.ptr<cv::Point2f>(r);
		uchar *mask_ptr = mask.ptr<uchar>(r);
		for (int c = 0; c < flow.cols; ++c) {
			const cv::Point2f &flow_vector = flow_ptr[c];
			uchar &mask_value = mask_ptr[c];
			mask_value = (flow_vector.dot(flow_vector) < mag_sqr) ? 0 : 255;
		}
	}
}

