#pragma once

#include <opencv2/core/core.hpp>
#include <functional>

typedef std::function<bool(const ushort &label)> Filter;

/*Connected Component
Finds and labels all pixels connected to a source pixel

@param binary_image: A binary image with uchar as pixel type
@param pixel: A pixel in which we start the search from
@param label: A label to assign to all connected pixels
@out labeled_image: An image with ushort as pixel type
*/
void connected_component(const cv::Mat &binary_image, const cv::Point2i &pixel, const ushort &label, cv::Mat &labeled_image);

/*Connected Components
Finds and labels all pixels in a binary image

@param binary_image: A binary image with uchar as pixel type
@out max_label: The amount of labels in the scene
@out labeled_image: An image with ushort as pixel type
*/
void connected_components(const cv::Mat &binary_image, cv::Mat &labeled_image, ushort &max_label);

/*Iterative Connected Components
Finds and labels all pixels in a binary image (much faster then just the connected_components algorithm)

@param binary_image: A binary image with uchar as pixel type
@out label_vector: A vector that contains all labels used
@out labeled_image: An image with ushort as pixel type
*/
void iterative_connected_components(const cv::Mat &binary_image, cv::Mat &labeled_image, std::vector<ushort> &label_vector);

// TODO
// COMMENT

void filter_labels(const cv::Mat &labeled_image, const ushort &labels, cv::Mat &relabeled_image, std::vector<ushort> &relabel_vector, Filter filter);

/*Condense Labels
Relabels labled_image so that all labels are sequencial. This function isn't neccisary but it might make it easier to debug.

@param label_vector: A vector of all labels in image
@param labeled_image: An labeled image
@out relabeled_image: A labeled image where all labels are sequencial
*/
void condense_labels(const std::vector<ushort> &label_vector, const cv::Mat &labeled_image, cv::Mat &relabeled_image);

/*Colorize Components
Index based coloring of an image based off the lable at each pixel. Background pixels are ignored.

@param labeled_image: An image with ushort as pixel type
@param label_colors: A vector of rgb colors
@out segmented_image: An image with cv::Vec3b as pixel type
*/
void colorize_components(const cv::Mat &labeled_image, const std::vector<cv::Vec3b> &color_vector, cv::Mat &segmented_image);

