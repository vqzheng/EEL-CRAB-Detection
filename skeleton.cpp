
#include "skeleton.h";


//input: a binary Mat
void find_skeleton(const cv::Mat &color, const cv::Mat& src, cv::Mat &res){
	src.copyTo(res);
	cv::Mat skel(res.size(), CV_8UC1, cv::Scalar(0));
	cv::Mat temp(res.size(), CV_8UC1);

	cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

	bool done;
	do
	{
		cv::morphologyEx(res, temp, cv::MORPH_OPEN, element);
		cv::bitwise_not(temp, temp);
		cv::bitwise_and(res, temp, temp);
		cv::bitwise_or(skel, temp, skel);
		cv::erode(res, res, element);

		double max;
		cv::minMaxLoc(res, 0, &max);
		done = (max == 0);
	} while (!done);

	res = color;

	for (int r = 0; r < src.rows; ++r) {
		const uchar *skel_ptr = skel.ptr<uchar>(r);
		cv::Vec3b *res_ptr = res.ptr<cv::Vec3b>(r);
		for (int c = 0; c < src.cols; ++c) {
			const uchar &skel_pixel = skel_ptr[c];
			cv::Vec3b &res_pixel = res_ptr[c];

			res_pixel[0] = (skel_pixel == 0) ? res_pixel[0] : 255;
			res_pixel[1] = (skel_pixel == 0) ? res_pixel[1] : 0;
			res_pixel[2] = (skel_pixel == 0) ? res_pixel[2] : 0;
		}
	}
}

