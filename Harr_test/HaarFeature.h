#ifndef _HaarFeature_H_
#define _HaarFeature_H_

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

namespace myColor
{
	const Scalar red = Scalar(0, 0, 255);
	const Scalar blue = Scalar(255, 0, 0);
};

struct HaarParams
{
	int widthMin;
	int widthMax;
	int widthStep;

	int ratioMin;
	int ratioMax;
};

class MyRect:public Rect
{
private:

public:
	int x;
	int y;
	int width;
	int height;
	Size size;
	Point2i center;
	Point2i topleft;
	Point2i topright;
	Point2i lowleft;
	Point2i lowright;

	// rectT��ǰ������center��
	MyRect(Rect rectT = Rect(100,100,50,50)) 
	{
		x = rectT.x;
		y = rectT.y;
		//�Զ���rectת��Ϊ����rray
		width = (rectT.width/2)*2+1; 
		height = (rectT.height/2)*2+1;

		size = Size(width,height);
		center = Point2i(x, y); 
		//�������ͨ���������ʵ��Խ��Լ��
		topleft = center - Point2i((width-1)/2, (height-1)/2);
		
		lowright = center + Point2i((width-1) / 2, (height-1) / 2);
		/*
		//�����Զ��޸�MyRect����Ч��Χ���Ӷ�����ʹ�����rect���⡣
		if (topleft.x < 0)
			topleft.x = 0;
		if (topleft.y < 0)
			topleft.y = 0;
		int rows = 620;
		int cols = 460;
		if (lowright.x >= cols)
			lowright.x = cols - 1;
		if (lowright.y >= rows)
			lowright.y = rows - 1;*/
		topright = Point2i(lowright.x,topleft.y);
		lowleft = Point2i(topleft.x, lowright.y);
	}
};

class HaarFeature
{
	
private:

public:
	// ˮƽ��ʽ��X����ֱ������Y;
	//width��smallRect�Ŀ��
	int x;
	int y;
	int width;
	int height;
	int ratio;

	int widthMin;
	int widthMax;
	int widthStep;
	int ratioMin;
	int ratioMax;

	MyRect bigRect;
	MyRect smallRect;
	MyRect bestBigRect;
	MyRect bestSmallRect;
	Mat_<int32_t> integralImg;

	HaarFeature(Mat_<uchar>& imgGray,HaarParams params)
	{
		ratioMin = params.ratioMin;
		ratioMax = params.ratioMax;
		widthMin = params.widthMin;
		widthMax = params.widthMax;
		widthStep = params.widthStep;

		ratio = ratioMin;
		width = (widthMin + widthMax) / 2;
		height = width;
		x = imgGray.cols / 2;
		y = imgGray.rows / 2;

		cout << "ratioMin=" << ratioMin << "	ratioMax=" << ratioMax<<endl;
		cout << "widthMin=" << widthMin << "	widthMax=" << widthMax;
		cout << "	widthStep=" << widthStep;

		smallRect = MyRect(Rect(x, y, width, height));
		bigRect = MyRect(Rect(x, y, width*ratio, height*ratio));

		//clock_t t=clock();
		integralImg = Mat_<int32_t>::zeros(imgGray.rows, imgGray.cols);
		integralImage(imgGray);
		//t = clock() - t;
		//cout << "\n" "elapseTime = " << double(t) / CLOCKS_PER_SEC * 1000 << "ms" << endl;
	};


	void integralImage(Mat_<uchar>& img);
	int32_t blockIntegral(MyRect rect);

	//����ĳ����p��Haar response,�ⲿ���ڲ�rect�ľ�ֵ��
	double HaarResponsePoint(Point2i p);
	//����ĳ��Haar kernel��response map
	double HaarResponseMap(Mat_<double>& responseMap);
	//�����������µ����response
	double HaarResponseMax();
	void haarShow(Mat& img, double maxf);

	//���أ�ʵʱ��ʾ�м��״̬�����ڵ���
	double HaarResponseMap_D(Mat_<double>& responseMap);
	double HaarResponseMax_D(Mat& img);
	void haarShow_D(Mat& img2, double f);
};


#endif