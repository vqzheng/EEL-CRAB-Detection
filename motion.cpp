#include "motion.h"

void energy(const cv::Mat &src, cv::Mat &dst, uchar buffer) {

	const uchar low_value = 255 - buffer;

	for (size_t r = 0; r < src.rows; ++r) {
		const uchar *src_row = src.ptr<uchar>(r);
		uchar *dst_row = dst.ptr<uchar>(r);

		for (size_t c = 0; c < src.cols; ++c) {
			const uchar &src_pixel = src_row[c];
			uchar &dst_pixel = dst_row[c];

			if (src_pixel > 0) {
				dst_pixel = 255;
			}
			else if (dst_pixel > low_value) {
				dst_pixel--;
			}
			else if (dst_pixel == low_value) {
				dst_pixel = 0;
			}
		}
	}
}