#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include "HaarFeature.h"

using namespace std;
using namespace cv;

void img2Gray(Mat& img, Mat& imgGray);
void HaarResponseMapDebug(Mat_<uchar>& imgGray, HaarParams& params)
{
	params.ratioMax = 2;
	params.widthMin = 123;
	params.widthMax = 123;
	HaarFeature Haar(imgGray, params);
	Mat_<double> responseMap;
	Haar.HaarResponseMap_D(responseMap);
}

int main()
{
	//----------------------------1 文件import + 参数设置---------------
	//string imgDir = "C:/KernelData/0 code lib/1_dataSets/Swiriski DataSets/p1-left"
		"/frames/600-eye.png";
	string imgDir = "C:/KernelData/0 code lib/1_dataSets/pupilGlint test data/image"
		"/NO1_01.png";
	Mat img = imread(imgDir);
	Mat_<uchar> imgGray;
	img2Gray(img, imgGray);
	
	HaarParams params;
	params.ratioMin = 2;
	params.ratioMax = 2;
	params.widthMin = 51;
	params.widthMax = 300;
	params.widthStep = 2;

	auto a = Mat_<int>::ones(4, 5);
	//Mat b = Mat_<int>::zeros(2, 3);
	//Mat_<int> c = (Mat_<int>(2,3)<<1,2,3,4,5,6);
	//c.col(0) = a.col(0) + b.col(0);
	//Mat b = a(Range(2, 1),Range(1,2));
	//cout << c(Point2i(0,1))<<"  "<<c(0,1)<<endl;

	for (int i =0;i<-10;i++)
		cout<< i<<endl;
	cout << a;

	//----------------------------2 cmd---------------
	time_t t=clock();
	HaarFeature Haar(imgGray,params);

	double maxf = Haar.HaarResponseMax();
	Haar.haarShow(img, maxf);

	// -------------debug--------------
	//double maxf = Haar.HaarResponseMax_D(img);
	HaarResponseMapDebug(imgGray,params);

	t = clock() - t;
	cout <<"\n" "elapseTime = "<<double(t)/CLOCKS_PER_SEC*1000<<"ms" << endl;

	system("pause");
	return 0;
}

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