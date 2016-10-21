#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdlib.h>
#include <stdio.h>



void dilation(const cv::Mat& src, cv::Mat &res, int size, cv::MorphShapes type);
void erosion(const cv::Mat& src, cv::Mat &res, int size, cv::MorphShapes type);