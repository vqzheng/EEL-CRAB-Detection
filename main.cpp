#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>

//using namespace cv;

int main(int argc, char** argv)
{
	const std::string source = ".../eel1.avi";
	cv::VideoCapture inputVideo(source);
	if (!inputVideo.isOpened()){
		std::cout << "Could not open the input video: " << source << std::endl;
		return -1;
	}
	cv::Mat raw;
	cv::namedWindow("source", CV_WINDOW_AUTOSIZE);

	for (;;){
		cv::Mat frame;
		inputVideo >> frame;
		cv::imshow("source", frame);
		if (cv::waitKey(30) >= 0) break;
	}
	


}