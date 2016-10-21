#include "object_analysis.h"
#include <cmath>
#include <algorithm>

void calculate_bounds(const cv::Mat &labeled_image, const ushort &labels, std::vector<cv::Rect2i> &bounds_vector) {
	std::vector<cv::Point2i> minpt_vector;
	minpt_vector.resize(labels);
	std::fill(minpt_vector.begin() + 1, minpt_vector.end(), cv::Point2i(labeled_image.cols, labeled_image.rows));

	std::vector<cv::Point2i> maxpt_vector;
	maxpt_vector.resize(labels);
	std::fill(maxpt_vector.begin() + 1, maxpt_vector.end(), cv::Point2i(0, 0));

	for (int r = 0; r < labeled_image.rows; ++r) {
		const ushort *labeled_image_ptr = labeled_image.ptr<ushort>(r);

		for (int c = 0; c < labeled_image.cols; ++c) {
			const ushort &labeled_image_pixel = labeled_image_ptr[c];
			if (labeled_image_pixel) {
				minpt_vector[labeled_image_pixel].x = (c < minpt_vector[labeled_image_pixel].x) ? c : minpt_vector[labeled_image_pixel].x;
				minpt_vector[labeled_image_pixel].y = (r < minpt_vector[labeled_image_pixel].y) ? r : minpt_vector[labeled_image_pixel].y;

				maxpt_vector[labeled_image_pixel].x = (c > maxpt_vector[labeled_image_pixel].x) ? c : maxpt_vector[labeled_image_pixel].x;
				maxpt_vector[labeled_image_pixel].y = (r > maxpt_vector[labeled_image_pixel].y) ? r : maxpt_vector[labeled_image_pixel].y;
			}
		}
	}

	bounds_vector.resize(labels);
	bounds_vector[0].x = 1;
	bounds_vector[0].y = 1;
	bounds_vector[0].width = labeled_image.cols - 2;
	bounds_vector[0].height = labeled_image.rows - 2;

	for (ushort l = 1; l < labels; ++l) {
		bounds_vector[l].x = minpt_vector[l].x;
		bounds_vector[l].y = minpt_vector[l].y;
		bounds_vector[l].width = maxpt_vector[l].x - minpt_vector[l].x;
		bounds_vector[l].height = maxpt_vector[l].y - minpt_vector[l].y;
	}
}

void calcualte_areas(const cv::Mat &labeled_image, const ushort &labels, std::vector<int> &area_vector) {
	area_vector.resize(labels);
	std::fill(area_vector.begin(), area_vector.end(), 0);

	for (int r = 0; r < labeled_image.rows; ++r) {
		const ushort *labeled_image_ptr = labeled_image.ptr<ushort>(r);

		for (int c = 0; c < labeled_image.cols; ++c) {
			const ushort &labeled_image_pixel = labeled_image_ptr[c];
			area_vector[labeled_image_pixel]++;
		}
	}
}

void calculate_perimeters(const cv::Mat &labeled_image, const ushort &labels, std::vector<int> &perimeter_vector){
	perimeter_vector.resize(labels);
	std::fill(perimeter_vector.begin(), perimeter_vector.end(), 0);

	for (int r = 1; r < labeled_image.rows - 1; ++r) {
		const ushort *labeled_image_ptr = labeled_image.ptr<ushort>(r);
		const ushort *labeled_image_north_ptr = labeled_image.ptr<ushort>(r + 1);
		const ushort *labeled_image_south_ptr = labeled_image.ptr<ushort>(r - 1);

		for (int c = 1; c < labeled_image.cols - 1; ++c) {
			const ushort &labeled_image_pixel = labeled_image_ptr[c];
			const ushort &labeled_image_north_pixel = labeled_image_north_ptr[c];
			const ushort &labeled_image_east_pixel = labeled_image_ptr[c + 1];
			const ushort &labeled_image_south_pixel = labeled_image_south_ptr[c];
			const ushort &labeled_image_west_pixel = labeled_image_ptr[c - 1];

			if ((labeled_image_north_pixel != labeled_image_pixel) ||
				(labeled_image_east_pixel != labeled_image_pixel) ||
				(labeled_image_south_pixel != labeled_image_pixel) ||
				(labeled_image_west_pixel != labeled_image_pixel)) {
				perimeter_vector[labeled_image_pixel];
			}

		}
	}
}

void calculate_area_perimeter_ratios(const std::vector<int> &area_vector, const std::vector<int> &perimeter_vector, std::vector<float> &ratio_vector) {
	ratio_vector.reserve(area_vector.size());
	for (ushort l = 0; l < area_vector.size(); ++l) {
		ratio_vector[l] = static_cast<float>(area_vector[l]) / static_cast<float>(perimeter_vector[l]);
	}
}

void calculate_compactness(const std::vector<int> &area_vector, const std::vector<int> &perimeter_vector, std::vector<float> &compactness_vector) {
	compactness_vector.reserve(area_vector.size());
	for (ushort l = 0; l < area_vector.size(); ++l) {
		compactness_vector[l] = static_cast<float>(perimeter_vector[l] * perimeter_vector[l]) / static_cast<float>(area_vector[l]);
	}
}

void calculate_centroids(const cv::Mat& labeled_image, const ushort &labels, std::vector<cv::Point2i> &centroid_vector) {
	
	centroid_vector.resize(labels);
	std::fill(centroid_vector.begin(), centroid_vector.end(), cv::Point2i(0, 0));

	std::vector<int> total_vector;
	total_vector.resize(labels);
	std::fill(total_vector.begin(), total_vector.end(), 0);

	for (int r = 0; r < labeled_image.rows; ++r) {
		const ushort *labeled_image_ptr = labeled_image.ptr<ushort>(r);

		for (int c = 0; c < labeled_image.cols; ++c) {
			const ushort &labeled_image_pixel = labeled_image_ptr[c];
			centroid_vector[labeled_image_pixel].x += c;
			centroid_vector[labeled_image_pixel].y += r;
			total_vector[labeled_image_pixel] ++;

		}
	}

	for (ushort l = 0; l < labels; ++l) {
		centroid_vector[l].x /= total_vector[l];
		centroid_vector[l].y /= total_vector[l];
	}
}

void precalculate_orientations(const cv::Mat& labeled_image, const ushort &labels, const std::vector<cv::Point2i> &centroid_vector, std::vector<int> &a_vector, std::vector<int> &b_vector, std::vector<int> &c_vector) {
	a_vector.resize(labels);
	std::fill(a_vector.begin(), a_vector.end(), 0);

	b_vector.resize(labels);
	std::fill(b_vector.begin(), b_vector.end(), 0);

	c_vector.resize(labels);
	std::fill(c_vector.begin(), c_vector.end(), 0);
	
	for (int r = 0; r < labeled_image.rows; ++r) {
		const ushort *labeled_image_ptr = labeled_image.ptr<ushort>(r);

		for (int c = 0; c < labeled_image.cols; ++c) {
			const ushort &labeled_image_pixel = labeled_image_ptr[c];
			const int transformed_c = c - centroid_vector[labeled_image_pixel].x;
			const int transformed_r = r - centroid_vector[labeled_image_pixel].y;

			a_vector[labeled_image_pixel] += (transformed_c * transformed_c);
			b_vector[labeled_image_pixel] += (2 * transformed_c * transformed_r);
			c_vector[labeled_image_pixel] += (transformed_r * transformed_r);
		}
	}
}

void calculate_orientations(const std::vector<int> &a_vector, const std::vector<int> &b_vector, const std::vector<int> &c_vector, std::vector<float> &alpha_vector, std::vector<int> &h_vector){
	h_vector.resize(a_vector.size());
	alpha_vector.resize(a_vector.size());
	
	for (ushort l = 0; l < a_vector.size(); ++l) {
		const long long a_minus_c = a_vector[l] - c_vector[l];
		const long long b_mul_b = b_vector[l] * b_vector[l];
		h_vector[l] = sqrt((a_minus_c * a_minus_c) + b_mul_b);
		alpha_vector[l] = atan(static_cast<float>(b_vector[l]) / static_cast<float>(a_minus_c)) / 2.0f;
	}
}

void calculate_circularity(const std::vector<int> &a_vector, const std::vector<int> &b_vector, const std::vector<int> &c_vector, const std::vector<int> &h_vector, std::vector<float> &emin_vector, std::vector<float> &emax_vector,std::vector<float> &circularity_vector) {
	emin_vector.resize(a_vector.size());
	emax_vector.resize(a_vector.size());
	circularity_vector.resize(a_vector.size());
	
	for (ushort l = 0; l < a_vector.size(); ++l) {
		const float h_div = 1.0f / static_cast<float>(h_vector[l]);
		const int a = a_vector[l];
		const int b = b_vector[l];
		const int c = c_vector[l];

		const float emin = ((a + c) - (a - c) * (a - c) * h_div - b * (b * h_div));
		const float emax = ((a + c) + (a - c) * (a - c) * h_div + b * (b * h_div));

		emin_vector[l] = emin / 2.0f;
		emax_vector[l] = emax / 2.0f;
		circularity_vector[l] = emin / emax;
	}
}
