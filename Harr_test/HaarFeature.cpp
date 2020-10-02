#include "HaarFeature.h"

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

/*
void HaarFeature::integralImage(Mat_<uchar>& img)
{
	//r=0行初始化
	Scalar_<int32_t> a;
	for (int c = 0; c < img.cols; c++)
	{
		a = sum(img(Range(0, 1), Range(0, c + 1)));
		integralImg(0, c) = a(0);
	}
	//r>0行初始化
	for (int r = 1; r < img.rows; r++)
	{
		for (int c = 0; c < img.cols; c++)
		{
			a = sum(img(Range(r, r + 1), Range(0, c + 1)));
			integralImg(r, c) = a(0) + integralImg(r - 1, c);
		}
	}
}*/

void HaarFeature::integralImage(Mat_<uchar>& img)
{
	integralImg.col(0) = img.col(0);
	for (int c = 1; c < img.cols; ++c)
		integralImg.col(c) = integralImg.col(c-1)+Mat_<int32_t>(img.col(c));
	for (int r = 1; r < img.rows; ++r)
		integralImg.row(r) += integralImg.row(r-1);

	/*//r=0行初始化
	Scalar_<int32_t> a;
	for (int c = 0; c < img.cols; c++)
	{
		a = sum(img(Range(0, 1), Range(0, c + 1)));
		integralImg(0, c) = a(0);
	}
	//r>0行初始化
	for (int r = 1; r < img.rows; r++)
	{
		for (int c = 0; c < img.cols; c++)
		{
			a = sum(img(Range(r, r + 1), Range(0, c + 1)));
			integralImg(r, c) = a(0) + integralImg(r - 1, c);
		}
	}*/
}

int32_t HaarFeature::blockIntegral(MyRect rect)
{
	int32_t bII = 0; //bII = block integral image
	if (rect.topleft.x > 0)
	{
		if (rect.topleft.y > 0)
			bII = integralImg(rect.lowright) - integralImg(rect.topright - Point2i(1, 1))\
			- integralImg(rect.lowleft - Point2i(1, 1)) + integralImg(rect.topleft - Point2i(1, 1));
		else if (rect.topleft.y == 0)
			bII = integralImg(rect.lowright) - integralImg(rect.lowleft - Point2i(1, 1));
	}
	else if (rect.topleft.x == 0)
	{
		if (rect.topleft.y > 0)
			bII = integralImg(rect.lowright) - integralImg(rect.topright - Point2i(1, 1));
		else if (rect.topleft.y == 0)
			bII = integralImg(rect.lowright);
		else
			cout << "topleft.y out" << endl;
	}
	else
		cout << "topleft.x out"<<endl;
	return bII;
}

double HaarFeature::HaarResponsePoint(Point2i p)
{
	MyRect smallRectT = MyRect(Rect(p.x, p.y, width, height));
	MyRect bigRectT = MyRect(Rect(p.x, p.y, width*ratio, height*ratio));

	double bIIBig = blockIntegral(bigRectT);
	double bIISmall = blockIntegral(smallRectT);
	int areaBig = bigRectT.width * bigRectT.height;
	int areaSmall = smallRectT.width * smallRectT.height;
	double f; //没有这步，下面会直接返回int
	return f = (bIIBig - bIISmall) / (areaBig - areaSmall) - bIISmall / areaSmall; //外部和内部rect的均值差
}

double HaarFeature::HaarResponseMap(Mat_<double>& responseMap)
{
	height = width;
	responseMap = Mat_<double>::zeros(integralImg.rows, integralImg.cols);
	double maxf = -250;
	int xiBest = 0;
	int yiBest = 0;
	MyRect rectT(Rect(x, y, width*ratio, height*ratio));//这步主要是获取width of big rect
	Range xRange = Range((rectT.width - 1) / 2, integralImg.cols - (rectT.width + 1) / 2);
	Range yRange = Range((rectT.height - 1) / 2, integralImg.rows - (rectT.height + 1) / 2);
	for (int xi = xRange.start; xi <= xRange.end; xi++)
		for (int yi = yRange.start; yi <= yRange.end; yi++)
		{
			responseMap(yi, xi) = HaarResponsePoint(Point2i(xi, yi));
			if (maxf < responseMap(yi, xi))
			{
				maxf = responseMap(yi, xi);
				xiBest = xi;
				yiBest = yi;
			}
		}
	smallRect = MyRect(Rect(xiBest, yiBest, width, height));
	bigRect = MyRect(Rect(xiBest, yiBest, width*ratio, height*ratio));

	return maxf;
}

double HaarFeature::HaarResponseMax()
{
	double f;
	double maxf = -250;
	Mat_<double> responseMap;

	for (ratio = ratioMin; ratio <= ratioMax; ratio++)
	{
		widthMax = min(widthMax, min(integralImg.rows, integralImg.cols) / ratio);
		for (width = widthMax; width >= widthMin; width -= widthStep)
		{
			height = width;
			f = HaarResponseMap(responseMap);
			if (maxf < f)
			{
				maxf = f;
				bestSmallRect = smallRect;
				bestBigRect = bigRect;
			}
		}
	}
	return maxf;
}

double HaarFeature::HaarResponseMap_D(Mat_<double>& responseMap)
{
	height = width;
	responseMap = Mat_<double>::zeros(integralImg.rows, integralImg.cols);
	double maxf = -250;
	int xiBest = 0;
	int yiBest = 0;
	MyRect rectT(Rect(x, y, width*ratio, height*ratio));//这步主要是获取width of big rect
	Range xRange = Range((rectT.width - 1) / 2, integralImg.cols - (rectT.width + 1) / 2);
	Range yRange = Range((rectT.height - 1) / 2, integralImg.rows - (rectT.height + 1) / 2);
	for (int xi = xRange.start; xi <= xRange.end; xi++)
		for (int yi = yRange.start; yi <= yRange.end; yi++)
		{
			responseMap(yi, xi) = HaarResponsePoint(Point2i(xi, yi));
			if (maxf < responseMap(yi, xi))
			{
				maxf = responseMap(yi, xi);
				xiBest = xi;
				yiBest = yi;
			}
		}
	smallRect = MyRect(Rect(xiBest, yiBest, width, height));
	bigRect = MyRect(Rect(xiBest, yiBest, width*ratio, height*ratio));

	//优化显示responseMap (hot map)
	double minT, maxT;
	minMaxLoc(responseMap, &minT, &maxT);
	Mat_<uchar> responseMap2 = Mat_<uchar>::zeros(integralImg.rows, integralImg.cols);
	responseMap2(yRange, xRange) = 255 / (maxT - minT)*(responseMap(yRange, xRange) - minT);

	Mat a;
	applyColorMap(responseMap2, a, COLORMAP_HOT);
	imshow("hot map", a);
	waitKey(30);

	return maxf;
}

double HaarFeature::HaarResponseMax_D(Mat& img)
{
	double f;
	double maxf = -250;
	Mat_<double> responseMap;

	ofstream fs("Harr");
	for (ratio = ratioMin; ratio <= ratioMax; ratio++)
	{
		fs << "ratio" << ratio << endl;
		fs << "width_f" << endl;
		widthMax = min(widthMax, min(integralImg.rows, integralImg.cols) / ratio);
		for (width = widthMax; width >= widthMin; width -= widthStep)
		{
			height = width;
			f = HaarResponseMap(responseMap);

			fs << fixed << setprecision(2);
			fs << smallRect.width << "  " << int(f * 100) / 100.0 << endl;
			haarShow_D(img, f); //

			if (maxf < f)
			{
				maxf = f;
				bestSmallRect = smallRect;
				bestBigRect = bigRect;
			}
		}
	}
	return maxf;
}

void HaarFeature::haarShow(Mat& img, double maxf)
{
	rectangle(img, bestSmallRect.topleft, bestSmallRect.lowright, myColor::red, 1, 8);
	rectangle(img, bestBigRect.topleft, bestBigRect.lowright, myColor::red, 1, 8);
	drawMarker(img, bestSmallRect.center, myColor::red);
	char buffer[10];
	sprintf_s(buffer,"%.2f",maxf);
	putText(img, buffer, smallRect.center, FONT_HERSHEY_SIMPLEX, 0.8, myColor::red);
	imshow("Haar", img);
	waitKey(30);
}

void HaarFeature::haarShow_D(Mat& img2, double f)
{
	Mat img = img2.clone();
	rectangle(img, smallRect.topleft, smallRect.lowright, myColor::red, 1, 8);
	rectangle(img, bigRect.topleft, bigRect.lowright, myColor::red, 1, 8);
	drawMarker(img, smallRect.center, myColor::red);
	char buffer[10];
	sprintf_s(buffer, "%.2f", f);
	putText(img, buffer, smallRect.center, FONT_HERSHEY_SIMPLEX, 0.8, myColor::red);
	imshow("Haar", img);
	waitKey(30);
}

