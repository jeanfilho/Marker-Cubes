#include <iostream>
#include <vector>
#include "opencv\cv.hpp"
#include "opencv\highgui.h"

int value = 0;
int type = 0;
bool use_adaptive = false;
int key = 0;
cv::VideoCapture vc(1);
cv::Mat frame, colour;
std::vector<std::vector<cv::Point>> contours;
std::vector<cv::Vec4i> hierarchy;
cv::RNG rng(12345);

void doThresholding()
{
	cv::cvtColor(frame, frame, CV_BGR2GRAY);
	if (use_adaptive)
	{
		cv::adaptiveThreshold(frame, frame, 255, value, type, 11, 2); //adjust to lighting 
	}
	else
	{
		cv::threshold(frame, frame, value, 255, type); //static - type 0 and 1 are BW - type 2,3 and 4 still keep the pixel value if they are above (?) the threshold 
	}
}

void doContours()
{
	colour = cv::Mat::zeros(frame.size(), CV_8UC3);
	cv::findContours(frame, contours, hierarchy, cv::RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	for (int idx = 0; idx < contours.size(); idx++)
		cv::approxPolyDP(contours[idx], contours[idx], 0.02, false);

	for (int idx = 0; idx < contours.size(); idx++)
	{
		cv::Scalar color(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		cv::drawContours(colour, contours, idx, color, 2,8);
	}
}

bool handleInput()
{
	key = cv::waitKey(60);
	if (key != 0)
	{
		if (key == 27)
			return true;
		if (key == 's')
		{
			use_adaptive = !use_adaptive;
			if (use_adaptive)
			{
				cv::setTrackbarMax("Parameter 2", "MarkerCubes", 1);
				cv::setTrackbarPos("Parameter 2", "MarkerCubes", 0);
				cv::setTrackbarMax("Parameter 1", "MarkerCubes", 1);
				cv::setTrackbarPos("Parameter 1", "MarkerCubes", 0);
			}
			else
			{
				cv::setTrackbarMax("Parameter 2", "MarkerCubes", 4);
				cv::setTrackbarPos("Parameter 2", "MarkerCubes", 0);
				cv::setTrackbarMax("Parameter 1", "MarkerCubes", 255);
				cv::setTrackbarPos("Parameter 1", "MarkerCubes", 0);
			}
		}

		return false;
	}
}

int main(int argc, char** argv)
{
	std::cout << "Starting program" << std::endl;


	cv::namedWindow("MarkerCubes");
	cv::createTrackbar("Parameter 1", "MarkerCubes", &value, 255);
	cv::createTrackbar("Parameter 2", "MarkerCubes", &type, 4);

	if (!vc.isOpened())
	{
		std::cout << "Webcam could not be started" << std::endl;
		return -1;
	}

	for (;;)
	{
		vc >> frame;
		doThresholding();
		doContours();


		cv::imshow("MarkerCubes", colour);
		if (handleInput())
			break;
	}

	return 0;
}
