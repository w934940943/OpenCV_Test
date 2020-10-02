#include <iostream>
#include <stdlib.h>
#include<thread>

#include <opencv2/opencv.hpp>

#include "PupilDetectorHaar.h"
#include <my_utilityfunctions.h>

using namespace std;
using namespace cv;


void imgTest()
{
	//----------------------------1 文件import + 参数设置 + preprocess---------------
	string imgDir = "C:/KernelData/0 code lib/1_dataSets/Swiriski DataSets/p1-left"
		"/frames/600-eye.png";
	//string imgDir = "C:/KernelData/0 code lib/1_dataSets/pupilGlint test data/image"
	//"/NO1_01.png";
	Mat img = imread(imgDir);
	if (img.empty()) { cout << "Cannot load img" << endl; /*return -1; */}

	HaarParams params;
	params.width_min = 51;
	params.width_max = 300;
	params.width_step = 1;
	params.ratio_min = 2;
	params.ratio_max = 3;

	Mat img_gray;
	img2Gray(img, img_gray);

	PupilDetectorHaar haar(img_gray, params);

}

void imgTestThread()
{
	//----------------------------1 文件import + 参数设置 + preprocess---------------
	string imgDir = "C:/KernelData/0 code lib/1_dataSets/Swiriski DataSets/p1-left"
		"/frames/600-eye.png";
	//string imgDir = "C:/KernelData/0 code lib/1_dataSets/pupilGlint test data/image"
	//"/NO1_01.png";
	Mat img = imread(imgDir);

	HaarParams params;
	params.width_min = 51;
	params.width_max = 300;
	params.width_step = 1;
	params.ratio_min = 2;
	params.ratio_max = 3;

	Mat img_gray;
	img2Gray(img, img_gray);

	//----------------------------2 cmd---------------
	//CTRL+F5, 直接F5运行时间差不少
	measureTime([&]() {
		thread thread1([&]() {
			measureTime([&]() {
				Mat img_gray2 = img_gray.clone();
				HaarParams params;
				params.width_min = 51;
				params.width_max = 300;
				params.width_step = 1;
				params.ratio_min = 2;
				params.ratio_max = 2;
				PupilDetectorHaar haar(img_gray2, params);
			}, "thread, ratio =2,3, without copy img  ", 1);
		});

		thread thread2([&]() {
			measureTime([&]() {
				//Mat img_gray2 = img_gray.clone();
				HaarParams params;
				params.width_min = 51;
				params.width_max = 300;
				params.width_step = 1;
				params.ratio_min = 3;
				params.ratio_max = 3;
				PupilDetectorHaar haar(img_gray, params);
			}, "thread, ratio =2,3, without copy img  ", 1);
		});
		thread1.join();
		thread2.join();
	}, "thread, ratio =2,3  ");

	measureTime([=]() {
		PupilDetectorHaar haar(img_gray, params);
		//cout << "迭代次数为：" << haar.iterate_count << endl;
	}, "no thread, ratio =2,3  ", 100);

}

void imgSequenceTest()
{
	string img_sequence = "C:/KernelData/0 code lib/1_dataSets/Swiriski DataSets/p1-left"
		"/frames/600-eye.png";
	cv::VideoCapture vc(img_sequence);

	if (!vc.isOpened()) {
		cerr << "Could not open " << img_sequence << endl;
	}
	else cout << "open success\n";

	Mat img;
	while (true)
	{
		vc >> img;
		if (img.empty())
			break;
		Mat img_gray;
		img2Gray(img, img_gray);

		HaarParams params;
		params.width_min = 51;
		params.width_max = 300;
		params.width_step = 4;
		params.ratio_min = 3;
		params.ratio_max = 3;
		PupilDetectorHaar haar;
		measureTime([&]() {
			haar.Detect(img_gray, params);
		});
		Mat img_pupil = Mat(img_gray, haar.pupil_rect_);
		imshowHist(img_pupil);
		imshowWithRect("PupilHaarDetection", img_gray, haar.pupil_rect_, haar.max_response_);
		waitKey(30);
	}
}

//--------------------- temporary test---------------------
void histTest()
{
	string imgDir = "C:/KernelData/0 code lib/1_dataSets/Swiriski DataSets/p1-left"
		"/frames/600-eye.png";
	Mat img = imread(imgDir);
	Mat img_gray;
	img2Gray(img, img_gray);
	imshowHist(img_gray); ;
}


int main()
{
	//haarDetection();
	imgSequenceTest();
	//histTest();
	
	//----------------------------------
	system("pause");
	return 0;
}



