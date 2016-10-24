#include "tank_detection.h"
#include <numeric>
#include "components.h"
#include "object_analysis.h"

bool detect_tank(const cv::Mat &hsv_image, cv::Rect2i &left_tank, cv::Rect2i &right_tank, const cv::Scalar &low_range, const cv::Scalar &high_range) {
	cv::Mat binary_image;
	cv::inRange(hsv_image, low_range, high_range, binary_image);

	cv::Mat labeled_image(binary_image.size(), CV_16U);
	std::vector<ushort> label_vector;
	iterative_connected_components(binary_image, labeled_image, label_vector);

	ushort labels = label_vector.size();
	ushort max_label = label_vector.back();
	condense_labels(label_vector, max_label, labeled_image, labeled_image);

	std::vector<int> area_vector;
	calcualte_areas(labeled_image, labels, area_vector);

	std::vector<cv::Rect> bounds_vector;
	calculate_bounds(labeled_image, labels, bounds_vector);

	std::vector<float> area_ratio_vector;
	area_ratio_vector.resize(labels);

	for (ushort l = 0; l < labels; ++l) {
		const float true_area = static_cast<float>(area_vector[l]);
		const float bounds_area = static_cast<float>(bounds_vector[l].area());		
		area_ratio_vector[l] = true_area * true_area / (1 + bounds_area);
	}

	if (labels < 3) {
		return false;
	}
	else {
		label_vector.resize(labels - 1);
		std::iota(label_vector.begin(), label_vector.end(), 1);
		std::sort(label_vector.begin(), label_vector.end(), [&](ushort i, ushort j) {return area_ratio_vector[j] < area_ratio_vector[i]; });

		const ushort i = label_vector[0];
		const ushort j = label_vector[1];

		left_tank = (bounds_vector[i].x < bounds_vector[j].x) ? bounds_vector[i] : bounds_vector[j];
		right_tank = (bounds_vector[i].x < bounds_vector[j].x) ? bounds_vector[j] : bounds_vector[i];
		return true;
	}
}
