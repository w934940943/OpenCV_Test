#ifndef PupilDetectorHaar_H_
#define PupilDetectorHaar_H_

#include <opencv2/opencv.hpp>

//#define UNIT_TEST

using namespace cv;
using namespace std;


namespace myColor
{
	const Scalar red = Scalar(0, 0, 255);
	const Scalar blue = Scalar(255, 0, 0);
};

struct HaarParams
{
	// width: --> pupil_rect
	int width_min;
	int width_max;
	int width_step;
	int ratio_min; // outer_rect ratio
	int ratio_max;
};


/* Detects pupil features using Haar detector.
@param imgGray input image: UINT8 [0, 255]. If imgGray is float [0,1], we can add a function to
	transfrom it to [0,255]. But it is not recommended, as [0,255] has better significance than [0,1].
*/
// TODO(zhwan): whether to use float image to compute
class PupilDetectorHaar
{
public:
	PupilDetectorHaar() :pupil_rect_(Rect()), outer_rect_(Rect()), max_response_(-255),iterate_count(0) {};
	PupilDetectorHaar(const Mat &imgGray, const HaarParams& params) : PupilDetectorHaar()
	{
		Detect(imgGray, params);
	}
	void Detect(const Mat &imgGray, const HaarParams& params);
	void show(const Mat& src, const Rect& pupil_rect, const Rect& outer_rect, \
		const float& max_response);

	Rect pupil_rect_;
	Rect outer_rect_;
	double max_response_;
	size_t iterate_count;

private:

	// imgGray: UINT8 [0,255]. float is not allowed.
	// intergral_img: a matrix, not a image, and its values are always large.So we 
	//   set its type <int>
	//void getIntegralImg(const Mat& imgGray, Mat& integral_img);

	double getResponseMap(const Mat &integral_img, int ratio, \
		int width, Rect& pupil_rect, Rect& outer_rect);

	/* Computes response value with some Haar kernel.

@param inner_rect is inputoutput.
@param outer_rect.
*/
	double getResponseValue(const Mat& integral_img, \
		Rect& inner_rect, Rect& outer_rect)
	{
		iterate_count += 1;
		// Filters rect, i.e., intersect two Rect.
		Rect a(0, 0, integral_img.cols - 1, integral_img.rows - 1);
		outer_rect &= a;
		inner_rect &= a;
		CV_Assert(outer_rect.width != 0);

		auto outer_bII = getBlockIntegral(integral_img, outer_rect);
		auto inner_bII = getBlockIntegral(integral_img, inner_rect);

		//外部和内部rect的均值差. integral_image采用int和double计算差不多.
		auto f = 1.0*(outer_bII - inner_bII) / (outer_rect.area() - \
			inner_rect.area()) - 1.0*inner_bII / inner_rect.area();
		return f;
	}

	/*
	@param rect must locate in the range of intergral_img.
	*/
	int getBlockIntegral(const Mat& integral_img, const Rect& rect)
	{
		/* integral_img
			  a(x1-1,y1-1)				   b (x2,y1-1)
						II(x1,y1)----------------
						|						|
						|		   Rect		  height
						|						|
			  c(x1-1,y2)---------width-------d (x2,y2)
		*/
		/* DEPRECATED
		int x1 = rect.x;
		int y1 = rect.y;
		int x2 = x1 + rect.width - 1;
		int y2 = y1 + rect.height - 1;
		int d = integral_img.at<int>(y2, x2);
		int b = y1 ? integral_img.at<int>(y1 - 1, x2) : 0;
		int c = x1 ? integral_img.at<int>(y2, x1 - 1) : 0;
		int a = x1 && y1 ? integral_img.at<int>(y1 - 1, x1 - 1) : 0;
		*/
		int d = integral_img.at<int32_t>(rect.y + rect.height, rect.x + rect.width);
		int c = integral_img.at<int32_t>(rect.y + rect.height, rect.x);
		int b = integral_img.at<int32_t>(rect.y, rect.x + rect.width);
		int a = integral_img.at<int32_t>(rect.y, rect.x);

		int integra_value = d + a - b - c;
		return integra_value;
	}
};

#endif