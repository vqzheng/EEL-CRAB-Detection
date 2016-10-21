#pragma once

#include <opencv2/core/core.hpp>

// TODO
// COMMENT

void calculate_bounds(const cv::Mat &labeled_image, const ushort &labels, std::vector<cv::Rect2i> &bounds_vector);

void calcualte_areas(const cv::Mat &labeled_image, const ushort &labels, std::vector<int> &area_vector);

void calculate_perimeters(const cv::Mat &labeled_image, const ushort &labels, std::vector<int> &perimeter_vector);

void calculate_area_perimeter_ratios(const std::vector<int> &area_vector, const std::vector<int> &perimeter_vector, std::vector<float> &ratio_vector);

void calculate_compactness(const std::vector<int> &area_vector, const std::vector<int> &perimeter_vector, std::vector<float> &compactness_vector);

void calculate_centroids(const cv::Mat& labeled_image, const ushort &labels, std::vector<cv::Point2i> &centroid_vector);

void precalculate_orientations(const cv::Mat& labeled_image, const ushort &labels, const std::vector<cv::Point2i> &centroid_vector, std::vector<int> &a_vector, std::vector<int> &b_vector, std::vector<int> &c_vector);

void calculate_orientations(const std::vector<int> &a_vector, const std::vector<int> &b_vector, const std::vector<int> &c_vector, std::vector<float> &alpha_vector, std::vector<int> &h_vector);

void calculate_circularity(const std::vector<int> &a_vector, const std::vector<int> &b_vector, const std::vector<int> &c_vector, const std::vector<int> &h_vector, std::vector<float> &emin_vector, std::vector<float> &emax_vector,std::vector<float> &circularity_vector);