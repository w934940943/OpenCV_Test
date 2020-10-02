#include<opencv2/opencv.hpp>
using namespace cv;

// ½«img×ª»»ÎªgrayÍ¼
void img2Gray(Mat& img, Mat& imgGray)
{
	if (img.channels() == 1)
		imgGray = img;
	else if (img.channels() == 3)
		cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);
	else if (img.channels() == 4)
		cv::cvtColor(img, imgGray, cv::COLOR_BGRA2GRAY);
	else
		throw std::runtime_error("Unsupported number of channels");
}