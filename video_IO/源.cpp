#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main()
{
	// test Scalar
	Scalar a = Scalar(1.0f,1.0f,1.0f);
	Mat b =Mat(3, 4, CV_16SC3, a);


	//1 ͼƬ��������ʾ
	Mat img = imread("C:\\KernelData\\0 code lib\\�����˱���+vive2.jpg");
	imshow("img", img);
	waitKey(100);
	
	//2 camera��������ʾ
	VideoCapture cap;
	cap.open(0);
	if (!cap.isOpened()) {
		cerr << "ERROR! Unable to open camera\n";
		return -1;
	}
	Mat frame;
	while (1)
	{
		cap.read(frame);
		imshow(" ", frame);
		if (waitKey(30) >= 0) break;
		//waitKey(0);//�ȴ��û�����
	}

	destroyAllWindows();
	return 0;

}