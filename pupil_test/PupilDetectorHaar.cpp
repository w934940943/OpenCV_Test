#include "PupilDetectorHaar.h"

#include <opencv2/opencv.hpp>
#include <my_utilityfunctions.h>

//#define UNIT_TEST
#define XYSTEP 4

using namespace cv;
using namespace std;


void PupilDetectorHaar::Detect(const Mat &imgGray, const HaarParams& params)
{
	//TODO(zhwan): guarantee the imgGray UIN8 [0,255], not float [0,1].
	CV_Assert(imgGray.depth() == DataType<uchar>::depth);

	Mat integral_img;
	integral(imgGray, integral_img); // size: (M+1)*(N+1)

	// Computes Haar response.
	int ratio_min = params.ratio_min;
	int ratio_max = params.ratio_max;
	int width_min = params.width_min;
	int wstep = params.width_step;
	for (int ratio = ratio_min; ratio <= ratio_max; ++ratio)
	{
#ifdef UNIT_TEST
		ofstream fs("Harr_output");
		fs << "ratio" << ratio << endl;
		fs << "width	response" << endl;
		fs << fixed << setprecision(2);
#endif
		// Decreses the search time.
		int width_max = min(params.width_max, min(imgGray.rows, imgGray.cols) / ratio);
		for (int width = width_min; width < width_max; width += wstep)
		{
			//cout<<"这个线程是："<<this_thread::get_id()<<endl;
			auto height = width;
			Rect pupil_rect, outer_rect;
			auto max_response = getResponseMap(integral_img, ratio, width, pupil_rect, outer_rect);
			if (max_response_ < max_response)
			{
				max_response_ = max_response;
				pupil_rect_ = pupil_rect;
				outer_rect_ = outer_rect;
			}
#ifdef UNIT_TEST
			show(imgGray, pupil_rect, outer_rect, max_response);
			fs << pupil_rect.width << "	" << max_response << endl;
#endif
		}
	}//end for ratio
#ifdef UNIT_TEST
	show(imgGray, pupil_rect_, outer_rect_, max_response_);
#endif
}


/* gets max Haar response.

@param pupil_rect output rect.
*/
double PupilDetectorHaar::getResponseMap(const Mat & integral_img, \
	int ratio, int width, Rect& pupil_rect, Rect& outer_rect)
{
	int height = width;
	//(x,y) is the lefttop corner of pupil_rect.
	//The range of (x,y) has two kinds: the first is the whole image, the second is the valid range.
	
#ifdef UNIT_TEST
	Mat response_map = Mat::zeros(integral_img.size(), CV_32F);
#endif
	decltype(max_response_) max_response = -255;
	//int xystep = 4;//macro definition is more convenient.
	auto xmax = integral_img.cols - 1 - width;
	auto ymax = integral_img.rows - 1 - height;
	for (int x = 0; x < xmax; x += XYSTEP)
		for (int y = 0; y < ymax; y += XYSTEP)
		{
			Rect pupil_rect0(x, y, width, height);
			Rect outer_rect0(x - (ratio - 1)*width / 2, y - (ratio - 1)*height / 2, \
				width*ratio, height*ratio);
			auto f = getResponseValue(integral_img, pupil_rect0, outer_rect0);
			if (max_response < f)
			{
				max_response = f;
				pupil_rect = pupil_rect0;
				outer_rect = outer_rect0;
			}
#ifdef UNIT_TEST
			response_map.at<float>(y + height / 2, x + width / 2) = f;
#endif
		}
#ifdef UNIT_TEST
	showHotMap(response_map);
#endif

	return max_response;
}


void PupilDetectorHaar::show(const Mat& src, const Rect& pupil_rect, \
	const Rect& outer_rect, const float& max_response)
{
	Mat img2;
	img2BGR(src, img2);

	rectangle(img2, pupil_rect, myColor::red, 1, 8);
	rectangle(img2, outer_rect, myColor::red, 1, 8);
	Point center = Point(pupil_rect.x + pupil_rect.width / 2, pupil_rect.y + pupil_rect.height / 2);
	drawMarker(img2, center, myColor::red);

	char buffer[10];
	sprintf_s(buffer, "%.2f", max_response);
	putText(img2, buffer, center, FONT_HERSHEY_SIMPLEX, 0.8, myColor::red);

	imshow("HaarFeatures", img2);
	waitKey(30);
}

