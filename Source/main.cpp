#include <iostream>
#include "opencv/cv.hpp"

int main(int argc, char** argv)
{
	std::cout << "Starting program" << std::endl;

	int value = 0;
	int type = 0;
	bool use_adaptive = false;
	int key = 0;
	cv::namedWindow("MarkerCubes");
	cv::createTrackbar("threshval", "MarkerCubes", &value, 255);
	cv::createTrackbar("threshtype", "MarkerCubes", &type, 4);

	cv::VideoCapture vc(0);
	if (!vc.isOpened())
	{
		std::cout << "Webcam could not be started" << std::endl;
		return -1;
	}
	cv::Mat frame;

	std::cout << "Entering main loop" << std::endl;
	for (;;)
	{
		vc >> frame;
		cv::cvtColor(frame, frame, CV_BGR2GRAY);
		if (use_adaptive)
			cv::adaptiveThreshold(frame, frame, 255, cv::ADAPTIVE_THRESH_MEAN_C, type, 11, 2);
		else
			cv::threshold(frame, frame, value, 255, type);
		

		cv::imshow("MarkerCubes", frame);

		key = cv::waitKey(30);
		if (key != 0)
		{
			if (key == 27)
				break;
			if (key == 's')
				use_adaptive = !use_adaptive;
		}
	}

	return 0;
}
