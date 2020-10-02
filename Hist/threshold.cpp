#include <opencv2/opencv.hpp>
#include <iostream>
using namespace std;
void sum_rgb2(const cv::Mat& src, cv::Mat& dst) {
	// Split image onto the color planes.
	//
	vector< cv::Mat> planes;
	cv::split(src, planes);
	cv::Mat b = planes[0], g = planes[1], r = planes[2], s;
	// Add equally weighted rgb values.
	//
	cv::addWeighted(r, 1. / 3., g, 1. / 3., 0.0, s);
	cv::addWeighted(s, 1., b, 1. / 3., 0.0, s);
	// Truncate values above 100.
//
	cv::threshold(s, dst, 100, 100, cv::THRESH_TRUNC);
}

void sum_rgb(const cv::Mat& src, cv::Mat& dst) {
	// Split image onto the color planes.
	//
	vector<cv::Mat> planes;
	cv::split(src, planes);
	cv::Mat b = planes[0], g = planes[1], r = planes[2];
	// Accumulate separate planes, combine and threshold.
	//
	cv::Mat s = cv::Mat::zeros(b.size(), CV_32F);
	cv::accumulate(b, s);
	cv::accumulate(g, s);
	cv::accumulate(r, s);
	// Truncate values above 100 and rescale into dst.
	//
	//double f = cv::threshold(s, s, 100, 100, cv::THRESH_OTSU);
	double f = cv::threshold(b, dst, 255, 255, cv::THRESH_OTSU | cv::THRESH_BINARY);
	cout <<"OTSU得到的最佳阈值"<< f << endl;
	//s.convertTo(dst, b.type());
}

void help() {
	cout << "Call: ./ch10_ex10_1 faceScene.jpg" << endl;
	cout << "Shows use of alpha blending (addWeighted) and threshold" << endl;
}

int main12(int argc, char** argv) {
	help();
	string imgDir = "C:/KernelData/0 code lib/1_dataSets/Swiriski DataSets/p1-left"
		"/frames/600-eye.png";
	//
	cv::Mat src = cv::imread(imgDir), dst;
	if (src.empty()) { cout << "can not load " << argv[1] << endl; return -1; }
	imshow("rawimg", src);
	sum_rgb(src, dst);
	// Create a named window with the name of the file and
	// show the image in the window
	//
	cv::imshow("threshold", dst);
	// Idle until the user hits any key.
	//
	cv::waitKey(0);
	return 0;
}