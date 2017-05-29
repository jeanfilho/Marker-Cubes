#include <opencv/highgui.h>
#include <opencv2\highgui\highgui.hpp>
#include "opencv2\opencv.hpp"
#include "opencv2\imgproc.hpp"
#include "opencv2\imgproc\imgproc.hpp"
#include <stdio.h>
#include <math.h>

using namespace cv;

#include <iostream>

using namespace std;


/// Initializes camera stream and returns error messages if necessary
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

/// Fetches a 2D-Int Array containing the marker values and returns a string with the marker id
string findSmallestMarkerID(int a[4][4])
{
	int results[4][4] = { {} };

	for (int rotations = 0; rotations < 4; rotations++)
	{
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				if (a[i][j] == 0)
				{
					switch (j) {
					case(0):
						results[rotations][i] += 8;
						break;
					case(1):
						results[rotations][i] += 4;
						break;
					case(2):
						results[rotations][i] += 2;
						break;
					case(3):
						results[rotations][i] += 1;
						break;
					default:
						break;
					}
				}
			}
		}

		//Rotate the whole matrix by 90 degrees to fetch all possible IDs
		int n = 4;
		int tmp;
		for (int i = 0; i<n / 2; i++) {
			for (int j = i; j<n - i - 1; j++) {
				tmp = a[i][j];
				a[i][j] = a[j][n - i - 1];
				a[j][n - i - 1] = a[n - i - 1][n - j - 1];
				a[n - i - 1][n - j - 1] = a[n - j - 1][i];
				a[n - j - 1][i] = tmp;
			}
		}
	}


	int lowest[4] = {};

	//Iterate through all possible results - get the orientation where the first line has the smallest sum
	for (int i = 0; i < 4; i++)
	{
		if (i == 0)
		{
			lowest[0] = results[0][0];
			lowest[1] = results[0][1];
			lowest[2] = results[0][2];
			lowest[3] = results[0][3];
		}
		else if (results[i][0] < lowest[0])
		{
			lowest[0] = results[i][0];
			lowest[1] = results[i][1];
			lowest[2] = results[i][2];
			lowest[3] = results[i][3];
		}
	}

	string res = "";

	for (int i = 0; i < 4; i++)
	{
		stringstream ss;
		ss << lowest[0] << "_" << lowest[1] << "_" << lowest[2] << "_" << lowest[3] << endl;
		res = ss.str();
	}

	return res;
}

/// Returns truth-Value of intersection from two functions as well as the point of intersection
bool intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2,
	Point2f &r)
{
	Point2f x = o2 - o1;
	Point2f d1 = p1 - o1;
	Point2f d2 = p2 - o2;

	float cross = d1.x*d2.y - d1.y*d2.x;
	if (abs(cross) < /*EPS*/1e-8)
		return false;

	double t1 = (x.x * d2.y - x.y * d2.x) / cross;
	r = o1 + d1 * t1;
	return true;
}

/// Takes a given matrix containing a transformed marker and returns its specific identification number
string checkMarkerID(Mat img)
{
	string result = "";

	if (img.rows != 6 || img.cols != 6)
	{
		return "NoMarker";
	}


	///Ignore outer borders that aren't black. Our markers always have black borders for contrast reasons
	for (int i = 0; i < 6; i++)
	{
		int pixel1 = img.at<uchar>(0, i);
		int pixel2 = img.at<uchar>(5, i);
		int pixel3 = img.at<uchar>(i, 0);
		int pixel4 = img.at<uchar>(i, 5);
		if ((pixel1 > 0) || (pixel2 > 0) || (pixel3 > 0) || (pixel4 > 0))
		{
			return "NoMarker";
		}
	}


	//Save and store the respective pixel values in a 2D Array
	int markerContent[6][6] = { {} };

	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			markerContent[i][j] = img.at<uchar>(i, j);

			if (markerContent[i][j] != 0)
			{
				markerContent[i][j] = 1;
			}
		}
	}

	//Remove all border values from our marker content arrays
	int reduxMarkerContent[4][4] = { {} };

	for (int i = 1; i < 5; i++)
	{
		for (int j = 1; j < 5; j++)
		{
			int tmp = markerContent[i][j];
			reduxMarkerContent[i - 1][j - 1] = tmp;
		}
	}

	//Send to helper function to make sure the smallest ID is found
	result = findSmallestMarkerID(reduxMarkerContent);

	return result;
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

	unsigned char* i = (unsigned char*)((pSrc.data + y * pSrc.step) + x);
	int a = i[0] + ((dx * (i[1] - i[0])) >> 8);
	i += pSrc.step;
	int b = i[0] + ((dx * (i[1] - i[0])) >> 8);
	return a + ((dy * (b - a)) >> 8);
}

int main(int ac, char** av)
{
	VideoCapture cap;

	const string kWinName1 = "Exercise 3 - Original Image";
	const string kWinName2 = "Exercise 3 - Converted Image";
	const string kWinName3 = "Exercise 4 - Marker Image Extracted";

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

			float lineParams[16];
			cv::Mat lineParamsMat(cv::Size(4, 4), CV_32F, lineParams); // lineParams is shared

			Point firstEdgePoints[2];
			vector<Point> lastEdgePoints = vector<Point>();
			Point2f subPixelCorners[4];

			for (int i = 0; i<4; ++i)
			{
				circle(img_bgr, rect[i], 3, Scalar(0, 255, 0), -1);
				Point currCornerPt = rect[i];

				//Difference in X and Y of this current point and the next point in polygon
				double dx = (double)(rect[(i + 1) % 4].x - rect[i].x) / 7.0;
				double dy = (double)(rect[(i + 1) % 4].y - rect[i].y) / 7.0;

				int stripeLength = (int)(0.8*sqrt(dx*dx + dy*dy));
				if (stripeLength < 5)
					stripeLength = 5;

				//make stripeLength odd (because of the shift in nStop)
				stripeLength = stripeLength | 1;

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


				vector<Point> inputPoints = vector<Point>();
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
								circle(img_bgr, p2, 1, CV_RGB(0, 0, 255), -1);

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

					points[j - 1].x = edgeCenter.x;
					points[j - 1].y = edgeCenter.y;

					if (isFirstStripe)
					{
						Mat iplTmp;
						resize(iplStripe, iplTmp, Size(100, 300));
						//imshow(kWinName3, iplTmp);
						isFirstStripe = false;
					}

					inputPoints.push_back(p);



				} // end of loop over edge points of one edge

				if (inputPoints.capacity() == 0)
				{
					continue;
				}


				vector<float> lineDrawn = vector<float>();
				fitLine(inputPoints, lineDrawn, CV_DIST_L2, 0, 0.01, 0.01);



				float lineDistance = (rect[1].x - rect[0].x) * 10;


				//Midpoint of fitted Line
				Point midPoint = Point(lineDrawn[2], lineDrawn[3]);
				//Direction Vector of fitted line
				Vec2f dir = Vec2f(lineDrawn[0], lineDrawn[1]);
				//Applying distance variable
				dir *= lineDistance;
				//Determining left and right border corners
				Point firstPoint = Point(midPoint.x + dir[0], midPoint.y + dir[1]);
				Point secondPoint = Point(midPoint.x - dir[0], midPoint.y - dir[1]);
				//Drawing the line
				line(img_bgr, firstPoint, secondPoint, CV_RGB(0, 255, 255), 1, CV_AA);

				inputPoints.clear();

				/*
				Until now, lines are just drawn - here: calculate intersections of all edges
				*/
				//First one doesn't have a precursor to work with
				if (lastEdgePoints.capacity() != 0)
				{
					Point2f intersect;
					intersection(lastEdgePoints[0], lastEdgePoints[1], firstPoint, secondPoint, intersect);
					circle(img_bgr, intersect, 3, CV_RGB(0, 255, 0), 3);
					subPixelCorners[i] = intersect;
				}

				else
				{
					firstEdgePoints[0] = firstPoint;
					firstEdgePoints[1] = secondPoint;
				}

				lastEdgePoints.clear();
				lastEdgePoints.push_back(firstPoint);
				lastEdgePoints.push_back(secondPoint);

				/*
				Find the last intersection between the last edge and the first edge
				*/
				if (i == 3)
				{
					Point2f intersect;
					intersection(firstEdgePoints[0], firstEdgePoints[1], firstPoint, secondPoint, intersect);
					circle(img_bgr, intersect, 3, CV_RGB(0, 255, 0), 3);
					subPixelCorners[0] = intersect;
				}


			} // end of loop over the 4 edges

			/*
			Define known distances between the points in the marker, so that the persp transf can retrieve the proper sized marker
			*/
			Point2f transformPts[4];
			transformPts[0] = Point2f(-0.5, -0.5);
			transformPts[1] = Point2f(-0.5, 5.5);
			transformPts[2] = Point2f(5.5, 5.5);
			transformPts[3] = Point2f(5.5, -0.5);
			Mat transformMat = getPerspectiveTransform(subPixelCorners, transformPts);

			Mat markerImage = Mat(Size(6, 6), CV_16SC3);
			warpPerspective(img_gray, markerImage, transformMat, Size(6, 6));

			threshold(markerImage, markerImage, 55, 255, CV_THRESH_BINARY);

			string id = checkMarkerID(markerImage);

			cout << id << endl;



			imshow(kWinName3, markerImage);

		} // end of loop over contours

		imshow(kWinName1, img_bgr);

		isFirstStripe = true;

		int key = cvWaitKey(10);
		if (key == 27) break;
	} // end of main loop
}