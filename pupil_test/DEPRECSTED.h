/* DEPRECATED
@param imgGray is input image (UINT8).
@param integral_img is output array (int).

void PupilDetectorHaar::getIntegralImg(const Mat& imgGray, Mat& integral_img)
{
	integral_img = Mat::zeros(imgGray.size(),CV_32S);
	Mat temp;
	imgGray.convertTo(temp, CV_32S);

	temp.col(0).copyTo(integral_img.col(0));
	for (int c = 1; c < imgGray.cols; ++c)
		integral_img.col(c) = integral_img.col(c - 1) + temp.col(c);
	for (int r = 1; r < imgGray.rows; ++r)
		integral_img.row(r) += integral_img.row(r - 1);
}*/