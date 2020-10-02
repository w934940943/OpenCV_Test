#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <plot.hpp>

// 本来是想测试一下openCV里面的plot的功能，但是给我的感觉是没有plot功能，可能是没有包括
using namespace std;
using namespace cv;

int main()
{
	Mat xData, yData, display;
	xData.create(1, 100, CV_64F);//1 Row, 100 columns, Double
	Ptr<plot::Plot2d> plot = cv::plot::Plot2d::create(xData);
	//Ptr<cv::plot::Plot2d> plot;
	/*
	yData.create(1, 100, CV_64F);

	for (int i = 0; i < 100; ++i)
	{
		xData.at<double>(i) = i / 10.0;
		yData.at<double>(i) = SQUARE(i / 10.0);
	}
	plot = plot::createPlot2d(xData, yData);
	plot->setPlotSize(100, 1000);
	plot->setMaxX(10);
	plot->setMinX(0);
	plot->setMaxY(100);
	plot->setMinY(-1);
	plot->render(display);
	imshow("Plot", display);
	waitKey();
	*/
	return 0;
}