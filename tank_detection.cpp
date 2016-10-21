#include "tank_detection.h"
#include <numeric>
#include "components.h"
#include "object_analysis.h"

void detect_tank(const cv::Mat &hsv_image, cv::Rect2i &left_tank, cv::Rect2i &right_tank, const cv::Scalar &low_range, const cv::Scalar &high_range) {
	cv::Mat binary_image;
	cv::inRange(hsv_image, low_range, high_range, binary_image);

	cv::Mat labeled_image(binary_image.size(), CV_16U);
	std::vector<ushort> label_vector;
	iterative_connected_components(binary_image, labeled_image, label_vector);

	ushort labels = label_vector.size();

	cv::Mat relabeled_image = cv::Mat::zeros(binary_image.size(), CV_16U);
	condense_labels(label_vector, labeled_image, relabeled_image);

	std::vector<int> area_vector;
	calcualte_areas(relabeled_image, labels, area_vector);

	std::vector<ushort> relabel_vector;
	relabel_vector.resize(labels - 1);
	std::iota(relabel_vector.begin(), relabel_vector.end(), 1);

	std::sort(relabel_vector.begin(), relabel_vector.end(), [&](ushort i, ushort j) {return area_vector[j] < area_vector[i]; });

	const ushort i = relabel_vector[0];
	const ushort j = relabel_vector[1];

	for (int r = 0; r < relabeled_image.rows; ++r) {
		ushort *relabeled_image_ptr = relabeled_image.ptr<ushort>(r);

		for (int c = 0; c < relabeled_image.cols; ++c) {
			ushort &relabeled_image_pixel = relabeled_image_ptr[c];
			relabeled_image_pixel = (relabeled_image_pixel == i) ? 1 : ((relabeled_image_pixel == j) ? 2 : 0);
		}
	}

	std::vector<cv::Rect2i> bounds_vector;
	calculate_bounds(relabeled_image, 3, bounds_vector);

	left_tank = (bounds_vector[1].x < bounds_vector[2].x) ? bounds_vector[1] : bounds_vector[2];
	right_tank = (bounds_vector[1].x < bounds_vector[2].x) ? bounds_vector[2] : bounds_vector[1];
}
