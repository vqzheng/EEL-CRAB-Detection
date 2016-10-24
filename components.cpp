#include "components.h"

#include <stack>
#include <vector>
#include <algorithm>
#include <functional>

void connected_component(const cv::Mat &binary_image, const cv::Point2i &pixel, const ushort &label, cv::Mat &labeled_image) {
	std::stack<cv::Point2i> pixels;
	pixels.push(pixel);

	const cv::Point2i directions[] {
		cv::Point2i(1, 0),
		cv::Point2i(-1, 0),
		cv::Point2i(0, 1),
		cv::Point2i(0, -1),
	};

	const cv::Rect2i bounds(cv::Point2i(0, 0), binary_image.size());
	
	while (!pixels.empty()) {
		const cv::Point2i &current_pixel = pixels.top();
		pixels.pop();

		for (const cv::Point2i &direction : directions) {
			const cv::Point2i neighbor_pixel = current_pixel + direction;
			
			if (bounds.contains(neighbor_pixel)) {
				if (binary_image.at<uchar>(neighbor_pixel) > 0 && 
					labeled_image.at<ushort>(neighbor_pixel) == 0) {

					labeled_image.at<ushort>(neighbor_pixel) = label;
					pixels.push(neighbor_pixel);
				}
			}
		}

	}
}

void connected_components(const cv::Mat &binary_image, cv::Mat &labeled_image, ushort &max_label) {
	ushort label = 1;

	for (int r = 0; r < binary_image.rows; ++r) {
		const uchar *binary_image_ptr = binary_image.ptr<uchar>(r);
		ushort *labeled_image_ptr = labeled_image.ptr<ushort>(r);

		for (int c = 0; c < binary_image.cols; ++c) {
			const uchar &binary_image_pixel = binary_image_ptr[c];
			ushort &labeled_image_pixel = labeled_image_ptr[c];

			if (binary_image_pixel != 0 && labeled_image_pixel == 0) {
				connected_component(binary_image, cv::Point2i(c, r), label++, labeled_image);
			}
		}
	}

	max_label = label;
}

void iterative_connected_components(const cv::Mat &binary_image, cv::Mat &labeled_image, std::vector<ushort> &label_vector) {
	ushort label = 1;
	labeled_image = 0;
	label_vector.clear();

	std::vector<ushort> parents;
	parents.push_back(0);

	for (int r = 1; r < binary_image.rows; ++r) {
		const uchar *binary_image_ptr = binary_image.ptr<uchar>(r);
		ushort *labeled_image_ptr = labeled_image.ptr<ushort>(r);
		ushort *south_labeled_image_ptr = labeled_image.ptr<ushort>(r - 1);

		for (int c = 1; c < binary_image.cols; ++c) {
			const uchar &binary_image_pixel = binary_image_ptr[c];
			ushort &labeled_image_pixel = labeled_image_ptr[c];
			ushort &labeled_image_pixel_west_neighbor = labeled_image_ptr[c - 1];
			ushort &labeled_image_pixel_south_neighbor = south_labeled_image_ptr[c];

			if (binary_image_pixel != 0) {
				if (labeled_image_pixel_west_neighbor == labeled_image_pixel_south_neighbor){
					if (labeled_image_pixel_west_neighbor == 0) {
						labeled_image_pixel = label++;
						parents.push_back(0);
					}
					else {
						labeled_image_pixel = labeled_image_pixel_west_neighbor;
					}
				}
				else if (labeled_image_pixel_west_neighbor == 0) {
					labeled_image_pixel = labeled_image_pixel_south_neighbor;
				}
				else if (labeled_image_pixel_south_neighbor == 0) {
					labeled_image_pixel = labeled_image_pixel_west_neighbor;
				}
				else if (labeled_image_pixel_west_neighbor < labeled_image_pixel_south_neighbor) {
					labeled_image_pixel = labeled_image_pixel_west_neighbor;
					parents[labeled_image_pixel_south_neighbor] = labeled_image_pixel;
				}
				else if (labeled_image_pixel_south_neighbor < labeled_image_pixel_west_neighbor) {
					labeled_image_pixel = labeled_image_pixel_south_neighbor;
					parents[labeled_image_pixel_west_neighbor] = labeled_image_pixel;
				}
			}
		}
	}

	for (int r = 1; r < labeled_image.rows; ++r) {
		ushort *labeled_image_ptr = labeled_image.ptr<ushort>(r);
		for (int c = 1; c < labeled_image.cols; ++c) {
			ushort &labeled_image_pixel = labeled_image_ptr[c];
			while (parents[labeled_image_pixel] != 0) {
				labeled_image_pixel = parents[labeled_image_pixel];
			}
		}
	}

	for (int p = 0; p < parents.size(); ++p) {
		if (parents[p] == 0) {
			label_vector.push_back(p);
		}
	}
}

void filter_labels(const cv::Mat &labeled_image, const ushort &labels, cv::Mat &relabeled_image, std::vector<ushort> &relabel_vector, Filter filter) {
	for (int r = 0; r < labeled_image.rows; ++r) {
		const ushort *labeled_image_ptr = labeled_image.ptr<ushort>(r);
		ushort *relabeled_image_ptr = relabeled_image.ptr<ushort>(r);

		for (int c = 0; c < labeled_image.cols; ++c) {
			const ushort &labeled_image_pixel = labeled_image_ptr[c];
			ushort &relabeled_image_pixel = relabeled_image_ptr[c];
			relabeled_image_pixel = filter(relabeled_image_pixel) ? relabeled_image_pixel : 0;
		}
	}

	relabel_vector.clear();
	relabel_vector.push_back(0);
	for (ushort l = 1; l < labels; ++l) {
		if (filter(l)) {
			relabel_vector.push_back(l);
		}
	}
}

void condense_labels(const std::vector<ushort> &label_vector, const ushort &max_label, const cv::Mat &labeled_image, cv::Mat &relabeled_image) {
	std::vector<ushort> relabel_vector;
	relabel_vector.resize(max_label + 1);

	for (ushort l = 0; l < label_vector.size(); ++l) {
		relabel_vector[label_vector[l]] = l;
	}

	for (int r = 1; r < labeled_image.rows; ++r) {
		const ushort *labeled_image_ptr = labeled_image.ptr<ushort>(r);
		ushort *relabeled_image_ptr = relabeled_image.ptr<ushort>(r);
		for (int c = 1; c < labeled_image.cols; ++c) {
			const ushort &labeled_image_pixel = labeled_image_ptr[c];
			ushort &relabeled_image_pixel = relabeled_image_ptr[c];
			relabeled_image_pixel = relabel_vector[labeled_image_pixel];
		}
	}
}

void colorize_components(const cv::Mat &labeled_image, const std::vector<cv::Vec3b> &color_vector, cv::Mat &segmented_image) {
	for (int r = 0; r < labeled_image.rows; ++r) {
		const ushort *labled_image_ptr = labeled_image.ptr<ushort>(r);
		cv::Vec3b *segmented_image_ptr = segmented_image.ptr<cv::Vec3b>(r);
		
		for (int c = 0; c < labeled_image.cols; ++c) {
			const ushort &labled_image_pixel = labled_image_ptr[c];
			cv::Vec3b &segmented_image_pixel = segmented_image_ptr[c];
			if (labled_image_pixel > 0) {
				segmented_image_pixel = color_vector[labled_image_pixel];
			}
		}
	}
}

