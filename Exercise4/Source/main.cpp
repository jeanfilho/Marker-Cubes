


#include <opencv/highgui.h>
#include "opencv2\opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>

using namespace cv;

#include <iostream>

using namespace std;


void initVideoStream(VideoCapture &cap) {
	if (cap.isOpened())
		cap.release();

	cap.open(1); // open the default camera
	if (cap.isOpened() == false) {
		cout << "No webcam found, using a video file" << endl;
		cap.open("MarkerMovie.mpg");
		if (cap.isOpened() == false) {
			cout << "No video file found. Exiting." << endl;
			exit(0);
		}
	}
}

int subpixSampleSafe(const Mat &pSrc, const Point2f &p)
{
	int x = int(floorf(p.x));
	int y = int(floorf(p.y));

	if (x < 0 || x >= pSrc.cols - 1 ||
		y < 0 || y >= pSrc.rows - 1)
		return 127;

	int dx = int(256 * (p.x - floorf(p.x)));
	int dy = int(256 * (p.y - floorf(p.y)));
	
	
	/*
	 * Idea: pixel value + influence from neighbors above and to the right
	 */
	unsigned char* i = (unsigned char*)((pSrc.data + y * pSrc.step) + x);
	int a = i[0] + ((dx * (i[1] - i[0])) >> 8); // Division by 256 -> normalized
	i += pSrc.step;
	int b = i[0] + ((dx * (i[1] - i[0])) >> 8);
	return a + ((dy * (b - a)) >> 8);
}

int main(int ac, char** av)
{
	VideoCapture cap;

	const string kWinName1 = "Exercise 3 - Original Image";
	const string kWinName2 = "Exercise 3 - Converted Image";
	const string kWinName3 = "Exercise 3 - Stripe Image";

	namedWindow(kWinName1, CV_WINDOW_AUTOSIZE);
	namedWindow(kWinName2, CV_WINDOW_NORMAL);
	namedWindow(kWinName3, CV_WINDOW_NORMAL);

	bool isFirstStripe = true;

	Mat img_bgr, img_gray, img_mono;

	initVideoStream(cap);

	for (;;) {

		cap >> img_bgr;

		if (img_bgr.empty()) {
			cout << "Could not query frame. Trying to reinitialize." << endl;
			initVideoStream(cap);
			// Wait for one sec.
			waitKey(1000);
			continue;
		}

		cvtColor(img_bgr, img_gray, CV_BGR2GRAY);

		adaptiveThreshold(img_gray, img_mono, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 11, 2);

		imshow(kWinName2, img_mono);

		vector<vector<Point>> contours, contour_Poly;

		Mat img_mono_(img_mono);

		findContours(img_mono, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

		contour_Poly.resize(contours.size());

		for (int i = 0; i < contours.size(); i++)
		{
			approxPolyDP(contours[i], contour_Poly[i], 3, true);

			if (contour_Poly[i].size() != 4) {
				continue;
			}

			Mat result_ = Mat(contour_Poly[i]);

			Rect r = boundingRect(result_);

			if (r.height<20 || r.width<20 || r.width > img_mono.cols - 10 || r.height > img_mono.rows - 10) {
				continue;
			}

			//Rect-Array contains all points of our current contour
			const Point *rect = (const Point*)result_.data;

			int npts = result_.rows;

			// draw the polygon
			polylines(img_bgr, &rect, &npts, 1,
				true,           // draw closed contour (i.e. joint end to start)
				CV_RGB(255, 0, 0),// colour RGB
				2,              // line thickness
				CV_AA);		// Draw ANTI-ALIASED Line

							//Go through our different edges
			for (int i = 0; i<4; ++i)
			{
				circle(img_bgr, rect[i], 3, Scalar(0, 255, 0), -1);

				//Difference in X and Y of this current point and the next point in polygon
				double dx = (double)(rect[(i + 1) % 4].x - rect[i].x) / 7.0;
				double dy = (double)(rect[(i + 1) % 4].y - rect[i].y) / 7.0;

				int stripeLength = (int)(0.8*sqrt(dx*dx + dy*dy));
				if (stripeLength < 5)
					stripeLength = 5;

				//make stripeLength odd (because of the shift in nStop)
				//stripeLength |= 1;
				stripeLength = stripeLength | 1;

				//e.g. stripeLength = 5 --> from -2 to 2
				// >> = Shift Right Operation (By 1 in this case)
				int nStop = stripeLength >> 1;
				int nStart = -nStop;

				Size stripeSize;
				stripeSize.width = 3;
				stripeSize.height = stripeLength;

				Point2f stripeVecX, stripeVecY;

				//normalize vectors
				double diffLength = sqrt(dx*dx + dy*dy);
				stripeVecX.x = dx / diffLength;
				stripeVecX.y = dy / diffLength;

				stripeVecY.x = stripeVecX.y;
				stripeVecY.y = -stripeVecX.x;

				Mat iplStripe(stripeSize, CV_8UC1);

				// Array for edge point centers
				Point2f points[6];

				for (int j = 1; j<7; ++j)
				{
					//Again: Rect-Array contains our different points of the current polygon
					double px = (double)rect[i].x + (double)j*dx;
					double py = (double)rect[i].y + (double)j*dy;

					Point p;
					p.x = (int)px;
					p.y = (int)py;
					circle(img_bgr, p, 2, CV_RGB(0, 0, 255), -1);

					for (int m = -1; m <= 1; ++m)
					{
						for (int n = nStart; n <= nStop; ++n)
						{
							Point2f subPixel;

							subPixel.x = (double)p.x + ((double)m * stripeVecX.x) + ((double)n * stripeVecY.x);
							subPixel.y = (double)p.y + ((double)m * stripeVecX.y) + ((double)n * stripeVecY.y);

							Point p2;

							p2.x = (int)subPixel.x;
							p2.y = (int)subPixel.y;

							if (isFirstStripe)
								circle(img_bgr, p2, 1, CV_RGB(255, 0, 255), -1);
							else
								circle(img_bgr, p2, 1, CV_RGB(0, 255, 255), -1);

							int pixel = subpixSampleSafe(img_gray, subPixel);

							int w = m + 1; //add 1 to shift to 0..2
							int h = n + (stripeLength >> 1); //add stripelenght>>1 to shift to 0..stripeLength

							iplStripe.at<uchar>(h, w) = (uchar)pixel;
						}
					}

					//use sobel operator on stripe
					// ( -1 , -2, -1 )
					// (  0 ,  0,  0 )
					// (  1 ,  2,  1 )

					vector<double> sobelValues(stripeLength - 2);
					///	double* sobelValues = new double[stripeLength-2];
					for (int n = 1; n < (stripeLength - 1); n++)
					{
						unsigned char* stripePtr = &(iplStripe.at<uchar>(n - 1, 0));
						///	unsigned char* stripePtr = ( unsigned char* )( iplStripe->imageData + (n-1) * iplStripe->widthStep );
						double r1 = -stripePtr[0] - 2 * stripePtr[1] - stripePtr[2];

						stripePtr += 2 * iplStripe.step;
						///	stripePtr += 2*iplStripe->widthStep;
						double r3 = stripePtr[0] + 2 * stripePtr[1] + stripePtr[2];
						sobelValues[n - 1] = r1 + r3;
					}

					double maxVal = -1;
					int maxIndex = 0;

					for (int n = 0; n<stripeLength - 2; ++n)
					{
						if (sobelValues[n] > maxVal)
						{
							maxVal = sobelValues[n];
							maxIndex = n;
						}
					}

					double y0, y1, y2; // y0 .. y1 .. y2
					y0 = (maxIndex <= 0) ? 0 : sobelValues[maxIndex - 1];
					y1 = sobelValues[maxIndex];
					y2 = (maxIndex >= stripeLength - 3) ? 0 : sobelValues[maxIndex + 1];

					//formula for calculating the x-coordinate of the vertex of a parabola, given 3 points with equal distances 
					//(xv means the x value of the vertex, d the distance between the points): 
					//xv = x1 + (d / 2) * (y2 - y0)/(2*y1 - y0 - y2)

					double pos = (y2 - y0) / (4 * y1 - 2 * y0 - 2 * y2); //d = 1 because of the normalization and x1 will be added later

																		 // This would be a valid check, too
																		 //if (isinf(pos)) {
																		 //	// value is infinity
																		 //	continue;
																		 //}

					if (pos != pos) {
						// value is not a number
						continue;
					}

					Point2f edgeCenter; //exact point with subpixel accuracy
					int maxIndexShift = maxIndex - (stripeLength >> 1);

					//shift the original edgepoint accordingly
					edgeCenter.x = (double)p.x + (((double)maxIndexShift + pos) * stripeVecY.x);
					edgeCenter.y = (double)p.y + (((double)maxIndexShift + pos) * stripeVecY.y);

					if (isFirstStripe)
					{
						Mat iplTmp;
						resize(iplStripe, iplTmp, Size(100, 300));
						imshow(kWinName3, iplTmp);
						isFirstStripe = false;
					}

				} // end of loop over edge points of one edge

			} // end of loop over the 4 edges

		} // end of loop over contours

		imshow(kWinName1, img_bgr);

		isFirstStripe = true;

		int key = cvWaitKey(10);
		if (key == 27) break;
	} // end of main loop
}