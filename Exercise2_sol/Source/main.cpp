// AR_Sheet2.cpp : Defines the entry point for the console application.
//
#include "opencv2\opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>

using namespace cv;


#include <iostream>

using namespace std;

int threshVal = 100;
int maxThresh = 255;
Mat frame, thresh, gray, drawing, canny, rects;
VideoCapture cap;
vector<vector<Point>> contour_Arr;

//Used for random operations like color-finding
RNG rng(12345);

/*
Takes X and Y coordinates of two 2D points and returns a large amount of points between those two
*/
vector<Point> linePoints(int x0, int y0, int x1, int y1)
{
	vector<Point> pointsOfLine;

	int dx = abs(x1 - x0), sx = x0<x1 ? 1 : -1;
	int dy = abs(y1 - y0), sy = y0<y1 ? 1 : -1;
	int err = (dx>dy ? dx : -dy) / 2, e2;

	for (;;)
	{
		pointsOfLine.push_back(Point(x0, y0));
		if (x0 == x1 && y0 == y1) break;
		e2 = err;
		if (e2 >-dx)
		{
			err -= dy;
			x0 += sx;
		}
		if (e2 < dy)
		{
			err += dx;
			y0 += sy;
		}
	}
	return pointsOfLine;
}

/*
Pass the detected Quads as 2D-Point Vectors to this function to have them drawn onto a given Matrix/Material
*/
void drawCirclesOnQuads(Mat drawTo, vector<vector<Point>> detected_Quads)
{
	for (int i = 0; i < detected_Quads.size(); i++)
	{
		vector<Point> tmp = detected_Quads[i];

		//4 Corner Points
		Point a = tmp[0];
		Point b = tmp[1];
		Point c = tmp[2];
		Point d = tmp[3];

		//Different combinations of our corners
		vector<vector<Point>> cornerCombinations{ { a, b },{ b,c },{ c,d },{ d,a } };

		//Iterate through combinations
		for (int x = 0; x < cornerCombinations.size(); x++)
		{
			//For this edge, get both corners
			Point one = cornerCombinations[x][0];
			Point two = cornerCombinations[x][1];

			//Fetch all points between both corners
			vector<Point> pts = linePoints(one.x, one.y, two.x, two.y);

			//Divide the returned points to get a desired amount of circles on this edge
			int stepSize = pts.size() / 7;

			//Sometimes empty lines are returned, drop these
			if (stepSize <= 0)
			{
				continue;
			}

			else
			{
				for (int j = 0; j < pts.size(); j++)
				{
					if (j % stepSize == 0)
					{
						//Draws the circle onto our Material/Matrix. Color Green(0,255,0).
						circle(drawTo, pts[j], 3, Scalar(0, 255, 0), 1);
					}
				}
			}
		}
	}
}


int main()
{
	//Create windows to hold our content
	namedWindow("Color", CV_GUI_NORMAL);
	namedWindow("Canny", CV_GUI_NORMAL);
	namedWindow("Thresh", CV_GUI_NORMAL);
	namedWindow("Contours", CV_GUI_NORMAL);

	//Open Video Stream. If you don't want to use a camera you can use imread() as input.
	cap.open(0);

	for (;;)
	{
		//Fetch next frame
		cap >> frame;
		//Resize for lesser CPU/GPU load
		resize(frame, frame, Size(640, 480));

		cvtColor(frame, gray, CV_BGR2GRAY);

		//Adjustable HighGUI-Trackbar to change threshhold during runtime
		createTrackbar("Threshold", "Thresh", &threshVal, maxThresh, NULL);

		threshold(gray, thresh, threshVal, maxThresh, THRESH_BINARY);

		imshow("Thresh", thresh);

		//Edge Detection using Canny's algorithm (Low Error Rate, Only one response from each edge, good localization of detected edges)
		Canny(thresh, canny, 100, 200, 3);

		imshow("Canny", canny);

		//Search for contours and load them into 2D Point Array.
		//CV_RETR_EXTERNAL returns outer shells of our shapes
		//CV_CHAIN_APPROX_SIMPLE drops "vertices" already covered by lines
		findContours(canny, contour_Arr, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

		drawing = Mat::zeros(frame.size(), CV_8SC3);

		//Iterate over array and draw all contours that belong TOGETHER in ONE color
		for (int i = 0; i < contour_Arr.size(); i++)
		{
			Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
			drawContours(drawing, contour_Arr, i, color, 2);
		}

		imshow("Contours", drawing);

		//Start simplifying towards polygons

		//Stores our polygons
		vector<vector<Point>> contours_poly(contour_Arr.size());
		//Our Axis-Aligned Bounding Rectangles
		vector<Rect> boundRect(contour_Arr.size());
		//And our final simplified Quads found in the image
		vector<vector<Point>> detected_Quads = vector<vector<Point>>();


		//Iterate through contours and reduce their polygons whereever possible
		for (int i = 0; i < contour_Arr.size(); i++)
		{
			approxPolyDP(contour_Arr[i], contours_poly[i], 3, true);
			boundRect[i] = boundingRect(contours_poly[i]);

			//Store polygons with 4 "vertices" as Quads. Line-Drawing for better presentation
			if (contours_poly[i].size() == 4)
			{
				rectangle(frame, boundRect[i], Scalar(255, 0, 0));
				polylines(frame, contours_poly[i], true, Scalar(0, 0, 255), 2);
				detected_Quads.push_back(contours_poly[i]);
			}

		}

		//Now we draw circles along our Quad-Edges. Separate function to maintain overview.
		drawCirclesOnQuads(frame, detected_Quads);

		imshow("Color", frame);

		//You can always cancel the code by pressing ESCAPE
		if (waitKey(10) == 27)
		{
			break;
		}
	}

	//Don't forget to destroy our windows
	return 0;
}