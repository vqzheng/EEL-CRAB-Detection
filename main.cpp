#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>

//using namespace cv;

int main(int argc, char** argv)
{
	const std::string source = "eel1.avi";
	cv::VideoCapture inputVideo(source);
	if (!inputVideo.isOpened()){
		std::cout << "Could not open the input video: " << source << std::endl;
		return -1;
	}
	cv::Mat edge;
	cv::namedWindow("source", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("edge", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("test", CV_WINDOW_AUTOSIZE);
	cv::Ptr<cv::BackgroundSubtractor> pMOG2 = cv::createBackgroundSubtractorMOG2();

	for (;;){
		cv::Mat frame;
		inputVideo >> frame;
		cvtColor(frame, edge, CV_BGR2GRAY);
		cv::GaussianBlur(edge, edge, cv::Size(9, 9), 1.5, 1.5);
		pMOG2->apply(edge, frame);
		cv::Canny(edge, edge, 0, 45, 3);
		cv::imshow("source", frame);
		cv::imshow("edge", edge);

		if (cv::waitKey(30) >= 0) break;
	}
	


}