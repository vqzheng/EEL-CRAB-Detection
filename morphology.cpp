/*Patrick Bellis, Arjun Lamba, Qiwei Zheng
10/02/2016
CS585 Assignment 3
*/
#include "morphology.h"

//dilation using opencv. parameter: input, output, size of the mask, type of mask(Morph_Rect, Morph_cross, Morph_ellipse)
void dilation(const cv::Mat& src, cv::Mat &res, int size, cv::MorphShapes type){
	cv::Mat element = cv::getStructuringElement(type, cv::Size(2 * size + 1, 2 * size + 1), cv::Point(size, size));
	cv::dilate(src, res, element);
}

//erosion using opencv. parameter: input, output, size of the mask, type of mask(Morph_Rect, Morph_cross, Morph_ellipse)
void erosion(const cv::Mat& src, cv::Mat &res, int size, cv::MorphShapes type){
	cv::Mat element = cv::getStructuringElement(type, cv::Size(2 * size + 1, 2 * size + 1), cv::Point(size, size));
	cv::erode(src, res, element);
}