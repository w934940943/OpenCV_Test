#include<opencv2/opencv.hpp>
#include<my_utilityfunctions.h>

using namespace std;
using namespace cv;



int main()
{
	Mat img = imread("../../../datasets/eye2.png");
	checkImg(img);

	uchar a = saturate_cast<uchar> (300) ;
	Mat img_gray;
	img2Gray(img, img_gray);

	measureTime([&]() {
		my_canny(img_gray); 
	});

	
	measureTime([&]() {
		Mat dst;
		double thresh_dl = 15;
		cv::Canny(img_gray, dst, thresh_dl, thresh_dl * 2, 3, true);
	});
	
	
	//vector<int> a;
	//for (auto i = 1; i < 100; ++i)
	//	a.push_back(i);
	

	return system("pause");
}


