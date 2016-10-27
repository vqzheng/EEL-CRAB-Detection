
#include "skeleton.h";


//input: a binary Mat
void find_skeleton(const cv::Mat& src, cv::Mat &res){
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

	for (int r = 0; r < src.rows; ++r) {
		const uchar *skel_ptr = skel.ptr<uchar>(r);
		uchar *res_ptr = res.ptr<uchar>(r);
		for (int c = 0; c < src.cols; ++c) {
			const uchar &skel_pixel = skel_ptr[c];
			uchar &res_pixel = res_ptr[c];

			res_pixel = (skel_pixel == 0) ? 0 : 255;
		}
	}
}

