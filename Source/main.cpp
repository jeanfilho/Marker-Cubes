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
	cv::createTrackbar("Parameter 1", "MarkerCubes", &value, 255);
	cv::createTrackbar("Parameter 2", "MarkerCubes", &type, 4);

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
		{
			cv::adaptiveThreshold(frame, frame, 255, value, type, 11, 2); //adjust to lighting 
		}
		else
		{
			cv::threshold(frame, frame, value, 255, type); //static - type 0 and 1 are BW - type 2,3 and 4 still keep the pixel value if they are above (?) the threshold 
		}
		

		cv::imshow("MarkerCubes", frame);

		key = cv::waitKey(30);
		if (key != 0)
		{
			if (key == 27)
				break;
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
		}
	}

	return 0;
}
