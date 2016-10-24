#pragma once
#include <opencv2/opencv.hpp>

bool detect_tank(const cv::Mat &hsv_image, cv::Rect2i &left_tank, cv::Rect2i &right_tank, const cv::Scalar &low_range = cv::Scalar(50, 0, 125), const cv::Scalar &high_range = cv::Scalar(125, 50, 255));
