#pragma once
#ifndef STEREORECONSTRUCTOR_H_
#define STEREORECONSTRUCTOR_H_


#include<opencv2/opencv.hpp>


using namespace std;
using namespace cv;

class StereoReconstructor
{
public:
	StereoReconstructor()
	{
		//通用参数
		dirname = "C:/Users/w9349/OneDrive/document/mycodelib/stereo_calibration/data/";
		intrinsic_filename = "intrinsics.yml";
		extrinsic_filename = "extrinsics.yml";

		//双目标定所需参数
		imagelistfn = "stereo_calib.xml";
		boardSize = Size(8, 5);
		squareSize = 1.0;

		//双目重建所需参数
		disparity_filename = "disparity.jpg";
		point_cloud_filename = "point-cloud";
		alg = STEREO_SGBM; //stereo corresponding 算法选择
		scale = 1; //img缩放系数，意义不大
		img_size = Size(640, 480);
		cn = 1;
	};


	string imagelistfn;
	Size boardSize;
	float squareSize;



	string dirname;
	string intrinsic_filename, extrinsic_filename;
	string disparity_filename, point_cloud_filename;
	string img1_filename, img2_filename;
	float scale;
	Size img_size;
	Mat M1, D1, M2, D2;
	Mat R, T, R1, P1, R2, P2, Q;
	Mat map11, map12, map21, map22;
	Rect roi1, roi2;
	Ptr<StereoBM> bm;
	Ptr<StereoSGBM> sgbm;

    Mat projMat1, projMat2;

	int numberOfDisparities;
	int cn;

	enum { STEREO_BM = 0, STEREO_SGBM = 1, STEREO_HH = 2, STEREO_VAR = 3, STEREO_3WAY = 4 };
	int alg;

//	int init(const Mat& img)
//	{
//		img_size = img.size();
//		cn = img.channels();

//                readCamParams(); //比如在上步后面，需要用到img_size

//		initStereoMatcher();

//		return 1;
//	}


        int readCamParams()
	{
		if ((!intrinsic_filename.empty()) ^ (!extrinsic_filename.empty())) //非或
		{
			printf("Command-line parameter error: either both intrinsic and extrinsic parameters must be specified, or none of them (when the stereo pair is already rectified)\n");
			return -1;
		}

		FileStorage fs(dirname + intrinsic_filename, FileStorage::READ);
		if (!fs.isOpened())
		{
			printf("Failed to open file %s\n", intrinsic_filename.c_str());
			return -1;
		}

		fs["M1"] >> M1;
		fs["D1"] >> D1;
		fs["M2"] >> M2;
		fs["D2"] >> D2;

		M1 *= scale;
		M2 *= scale;


		fs.open(dirname + extrinsic_filename, FileStorage::READ);
		if (!fs.isOpened())
		{
			printf("Failed to open file %s\n", extrinsic_filename.c_str());
			return -1;
		}

		fs["R"] >> R;
		fs["T"] >> T;

                //用于双目dense重建的参数
//		stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);

//		initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
//		initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

		return 1;
	}

        /****************************************************
                          Sparse reconstruction
        ****************************************************/
        void initProjMats()
        {
            Mat R0 = Mat::eye(3, 3, CV_64F);
            Mat T0 = Mat::zeros(3, 1, CV_64F);
            Mat A1;
            hconcat(R0, T0, A1);
            projMat1 = M1*A1;

            Mat A2;
            hconcat(R, T, A2);
            projMat2 = M2*A2; //3*4
        }

        void initSparseReconstructParams()
        {
            readCamParams();
            initProjMats();
        }

        //pts1, 2*N
        // pts4D, 4*N
        void getSparsePoint(const Mat& pts1, const Mat& pts2, Mat& pts4D)
        {
            triangulatePoints(projMat1, projMat2, pts1, pts2, pts4D);
            pts4D = pts4D / pts4D.at<double>(3,0);
        }

        //测试sparse point cloud reconstruction

        void testSparse()
        {
                initSparseReconstructParams();
                Mat pts1 = (Mat_<double>(2, 1) << 401, 327);
                Mat pts2 = (Mat_<double>(2, 1) << 183, 304);
                Mat pts4D;
                getSparsePoint(pts1,pts2,pts4D);
        }



        /****************************************************
                          Dense reconstruction
        ****************************************************/
        void initDenseReconstructParams(const Mat& img)
        {
            readCamParams();

            img_size = img.size();
            cn = img.channels();

            //用于双目dense重建的参数
            stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);

            initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
            initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

            initStereoMatcher();
        }

	int initStereoMatcher()
	{
		//BM算法只能处理grayscale
		//SGBM算法可以处理color
		bm = StereoBM::create(16, 9);
		sgbm = StereoSGBM::create(0, 16, 3);
		int SADWindowSize = 5; //blocksize，正奇数 >= 1
		numberOfDisparities = 16*10; //比如是16的倍数, max-disparity
		if (numberOfDisparities < 1 || numberOfDisparities % 16 != 0)
		{
			printf("Command-line parameter error: The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16\n");
			return -1;
		}

		//&是按位与操作
		numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width / 8) + 15) & -16;

		bm->setROI1(roi1);
		bm->setROI2(roi2);
		bm->setPreFilterCap(31);
		bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
		bm->setMinDisparity(0);
		bm->setNumDisparities(numberOfDisparities);
		bm->setTextureThreshold(10);
		bm->setUniquenessRatio(15);
		bm->setSpeckleWindowSize(100);
		bm->setSpeckleRange(32);
		bm->setDisp12MaxDiff(1);

		sgbm->setPreFilterCap(63);
		int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
		sgbm->setBlockSize(sgbmWinSize);

		sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
		sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
		sgbm->setMinDisparity(0);
		sgbm->setNumDisparities(numberOfDisparities);
		sgbm->setUniquenessRatio(10);
		sgbm->setSpeckleWindowSize(100);
		sgbm->setSpeckleRange(32);
		sgbm->setDisp12MaxDiff(1);
		if (alg == STEREO_HH)
			sgbm->setMode(StereoSGBM::MODE_HH);
		else if (alg == STEREO_SGBM)
			sgbm->setMode(StereoSGBM::MODE_SGBM);
		else if (alg == STEREO_3WAY)
			sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);
	}

        void showDenseReconstruction(const Mat& img1, const Mat& img2, const Mat disp8)
	{
		//namedWindow("left", 1);
		imshow("left", img1);
		//namedWindow("right", 1);
		imshow("right", img2);
		//namedWindow("disparity", 0);
		imshow("disparity", disp8);
		printf("press any key to continue...");
		fflush(stdout);
		waitKey(30);
		printf("\n");
	}

	int stereoMatch(Mat& img1, Mat& img2, Mat& disp8)
	{
		//2 image pair rectification
		Mat img1r, img2r;
		remap(img1, img1r, map11, map12, INTER_LINEAR);
		remap(img2, img2r, map21, map22, INTER_LINEAR);

		img1 = img1r;
		img2 = img2r;

		//3 disparity map 计算
		Mat disp;
		//Mat img1p, img2p, dispp;
		//copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
		//copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);

		int64 t = getTickCount();
		if (alg == STEREO_BM)
			bm->compute(img1, img2, disp);
		else if (alg == STEREO_SGBM || alg == STEREO_HH || alg == STEREO_3WAY)
			sgbm->compute(img1, img2, disp);
		t = getTickCount() - t;
		printf("Time elapsed: %fms\n", t * 1000 / getTickFrequency());

		//disp = dispp.colRange(numberOfDisparities, img1p.cols);
		if (alg != STEREO_VAR)
			disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));
		else
			disp.convertTo(disp8, CV_8U);


		//4 point clound 计算
		Mat xyz;
		reprojectImageTo3D(disp, xyz, Q, true);
		//if (!point_cloud_filename.empty())
		//{
		//	printf("storing the point cloud...");
		//	fflush(stdout);
		//	//saveXYZ(point_cloud_filename.c_str(), xyz);
		//	printf("\n");
		//}

		return 0;
	}

	
	//OpenCV sample 原始文档，仅修改部分
	int stereoMatch()
	{
		//1 参数配置
		const string dirname = "C:/Users/w9349/OneDrive/document/mycodelib/stereo_calibration/data/";

		string img1_filename = dirname + "left01.jpg";
		string img2_filename = dirname + "right01.jpg";
		std::string intrinsic_filename = "intrinsics.yml";
		std::string extrinsic_filename = "extrinsics.yml";
		std::string disparity_filename = "disparity.jpg";
		std::string point_cloud_filename = "point-cloud";
		if ((!intrinsic_filename.empty()) ^ (!extrinsic_filename.empty()))
		{
			printf("Command-line parameter error: either both intrinsic and extrinsic parameters must be specified, or none of them (when the stereo pair is already rectified)\n");
			return -1;
		}
		if (extrinsic_filename.empty() && !point_cloud_filename.empty())
		{
			printf("Command-line parameter error: extrinsic and intrinsic parameters must be specified to compute the point cloud\n");
			return -1;
		}


		enum { STEREO_BM = 0, STEREO_SGBM = 1, STEREO_HH = 2, STEREO_VAR = 3, STEREO_3WAY = 4 };
		int alg = STEREO_SGBM; //stereo corresponding 算法选择

		Ptr<StereoBM> bm = StereoBM::create(16, 9);
		Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 16, 3);
		int SADWindowSize = 5; //blocksize，正奇数 >= 1
		int numberOfDisparities = 80; //比如是16的倍数, max-disparity
		if (numberOfDisparities < 1 || numberOfDisparities % 16 != 0)
		{
			printf("Command-line parameter error: The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16\n");
			return -1;
		}

		bool no_display = false; //是否显示最后的处理结果，[显示]

		int color_mode = alg == STEREO_BM ? 0 : -1;
		Mat img1 = imread(img1_filename, color_mode);
		Mat img2 = imread(img2_filename, color_mode);
		if (img1.empty())
		{
			printf("Command-line parameter error: could not load the first input image file\n");
			return -1;
		}
		if (img2.empty())
		{
			printf("Command-line parameter error: could not load the second input image file\n");
			return -1;
		}

		float scale = 1;
		if (scale != 1.f)
		{
			Mat temp1, temp2;
			int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
			resize(img1, temp1, Size(), scale, scale, method);
			img1 = temp1;
			resize(img2, temp2, Size(), scale, scale, method);
			img2 = temp2;
		}

		Size img_size = img1.size();

		Rect roi1, roi2;
		Mat Q;

		//2 image pair rectification
		if (!intrinsic_filename.empty() && !extrinsic_filename.empty())
		{
			// reading intrinsic parameters
			FileStorage fs(intrinsic_filename, FileStorage::READ);
			if (!fs.isOpened())
			{
				printf("Failed to open file %s\n", intrinsic_filename.c_str());
				return -1;
			}

			Mat M1, D1, M2, D2;
			fs["M1"] >> M1;
			fs["D1"] >> D1;
			fs["M2"] >> M2;
			fs["D2"] >> D2;

			M1 *= scale;
			M2 *= scale;

			fs.open(extrinsic_filename, FileStorage::READ);
			if (!fs.isOpened())
			{
				printf("Failed to open file %s\n", extrinsic_filename.c_str());
				return -1;
			}

			Mat R, T, R1, P1, R2, P2;
			fs["R"] >> R;
			fs["T"] >> T;

			stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);

			Mat map11, map12, map21, map22;
			initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
			initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

			Mat img1r, img2r;
			remap(img1, img1r, map11, map12, INTER_LINEAR);
			remap(img2, img2r, map21, map22, INTER_LINEAR);

			img1 = img1r;
			img2 = img2r;
		}

		//3 disparity map 计算
		//&是按位与操作
		numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width / 8) + 15) & -16;

		bm->setROI1(roi1);
		bm->setROI2(roi2);
		bm->setPreFilterCap(31);
		bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
		bm->setMinDisparity(0);
		bm->setNumDisparities(numberOfDisparities);
		bm->setTextureThreshold(10);
		bm->setUniquenessRatio(15);
		bm->setSpeckleWindowSize(100);
		bm->setSpeckleRange(32);
		bm->setDisp12MaxDiff(1);

		sgbm->setPreFilterCap(63);
		int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
		sgbm->setBlockSize(sgbmWinSize);

		int cn = img1.channels();

		sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
		sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
		sgbm->setMinDisparity(0);
		sgbm->setNumDisparities(numberOfDisparities);
		sgbm->setUniquenessRatio(10);
		sgbm->setSpeckleWindowSize(100);
		sgbm->setSpeckleRange(32);
		sgbm->setDisp12MaxDiff(1);
		if (alg == STEREO_HH)
			sgbm->setMode(StereoSGBM::MODE_HH);
		else if (alg == STEREO_SGBM)
			sgbm->setMode(StereoSGBM::MODE_SGBM);
		else if (alg == STEREO_3WAY)
			sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);



		Mat disp, disp8;
		//Mat img1p, img2p, dispp;
		//copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
		//copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);

		int64 t = getTickCount();
		if (alg == STEREO_BM)
			bm->compute(img1, img2, disp);
		else if (alg == STEREO_SGBM || alg == STEREO_HH || alg == STEREO_3WAY)
			sgbm->compute(img1, img2, disp);
		t = getTickCount() - t;
		printf("Time elapsed: %fms\n", t * 1000 / getTickFrequency());

		//disp = dispp.colRange(numberOfDisparities, img1p.cols);
		if (alg != STEREO_VAR)
			disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));
		else
			disp.convertTo(disp8, CV_8U);

		//结果显示
		if (!no_display)
		{
			namedWindow("left", 1);
			imshow("left", img1);
			namedWindow("right", 1);
			imshow("right", img2);
			namedWindow("disparity", 0);
			imshow("disparity", disp8);
			printf("press any key to continue...");
			fflush(stdout);
			waitKey(30);
			printf("\n");
		}


		//文件保存
		if (!disparity_filename.empty())
			imwrite(disparity_filename, disp8);

		//4 point clound 计算
		if (!point_cloud_filename.empty())
		{
			printf("storing the point cloud...");
			fflush(stdout);
			Mat xyz;
			reprojectImageTo3D(disp, xyz, Q, true);
			//saveXYZ(point_cloud_filename.c_str(), xyz);
			printf("\n");
		}

		return 0;
	}



	//灰度转伪彩色, 效果和image watch一致
	//cvtColor(disp8, tmp, COLOR_GRAY2BGR); 并不能实现该效果
	Mat gray2Color(Mat img)
	{
		Mat img_color(img.rows, img.cols, CV_8UC3);//构造RGB图像
#define IMG_B(img,y,x) img.at<Vec3b>(y,x)[0]
#define IMG_G(img,y,x) img.at<Vec3b>(y,x)[1]
#define IMG_R(img,y,x) img.at<Vec3b>(y,x)[2]
		uchar tmp2 = 0;
		//转为彩虹图的具体算法，主要思路是把灰度图对应的0～255的数值分别转换成彩虹色：红、橙、黄、绿、青、蓝。
		for (int y = 0; y < img.rows; y++)
		{
			for (int x = 0; x < img.cols; x++)
			{
				tmp2 = img.at<uchar>(y, x);
				if (tmp2 <= 51)
				{
					IMG_B(img_color, y, x) = 255;
					IMG_G(img_color, y, x) = tmp2 * 5;
					IMG_R(img_color, y, x) = 0;
				}
				else if (tmp2 <= 102)
				{
					tmp2 -= 51;
					IMG_B(img_color, y, x) = 255 - tmp2 * 5;
					IMG_G(img_color, y, x) = 255;
					IMG_R(img_color, y, x) = 0;
				}
				else if (tmp2 <= 153)
				{
					tmp2 -= 102;
					IMG_B(img_color, y, x) = 0;
					IMG_G(img_color, y, x) = 255;
					IMG_R(img_color, y, x) = tmp2 * 5;
				}
				else if (tmp2 <= 204)
				{
					tmp2 -= 153;
					IMG_B(img_color, y, x) = 0;
					IMG_G(img_color, y, x) = 255 - uchar(128.0*tmp2 / 51.0 + 0.5);
					IMG_R(img_color, y, x) = 255;
				}
				else
				{
					tmp2 -= 204;
					IMG_B(img_color, y, x) = 0;
					IMG_G(img_color, y, x) = 127 - uchar(127.0*tmp2 / 51.0 + 0.5);
					IMG_R(img_color, y, x) = 255;
				}
			}
		}
		return img_color;
	}



	//测试一对图片
        int testDense1()
	{
		//图片导入
		//string img1_filename = dirname + "left01.jpg";
		//string img2_filename = dirname + "right01.jpg";
		Mat img1 = imread(img1_filename);
		Mat img2 = imread(img2_filename);
		if (img1.empty())
		{
			printf("Command-line parameter error: could not load the first input image file\n");
			return -1;
		}
		if (img2.empty())
		{
			printf("Command-line parameter error: could not load the second input image file\n");
			return -1;
		}

		if (alg == STEREO_BM)
		{
			img2Gray(img1, img1);
			img2Gray(img2, img2);
		}

		if (scale != 1.f)
		{
			Mat temp1, temp2;
			int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
			resize(img1, temp1, Size(), scale, scale, method);
			img1 = temp1;
			resize(img2, temp2, Size(), scale, scale, method);
			img2 = temp2;
		}

                initDenseReconstructParams(img1);

		Mat disp8;
		stereoMatch(img1, img2, disp8);

                showDenseReconstruction(img1, img2, disp8);

		Mat tmp;
		cvtColor(disp8, tmp, COLOR_GRAY2BGR);
		Mat t = gray2Color(disp8);
		//文件保存
		//if (!disparity_filename.empty())
		//	imwrite(disparity_filename, disp8);
	}

	//测试图片序列
        int testDense2()
	{
		//图片导入
		string img1_filename = dirname + "left01.jpg";
		string img2_filename = dirname + "right01.jpg";
		Mat img1 = imread(img1_filename);
		Mat img2 = imread(img2_filename);
		if (img1.empty())
		{
			printf("Command-line parameter error: could not load the first input image file\n");
			return -1;
		}
		if (img2.empty())
		{
			printf("Command-line parameter error: could not load the second input image file\n");
			return -1;
		}

		if (alg == STEREO_BM)
		{
			img2Gray(img1, img1);
			img2Gray(img2, img2);
		}

		if (scale != 1.f)
		{
			Mat temp1, temp2;
			int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
			resize(img1, temp1, Size(), scale, scale, method);
			img1 = temp1;
			resize(img2, temp2, Size(), scale, scale, method);
			img2 = temp2;
		}

                initDenseReconstructParams(img1);


		for (int i = 1; i != 10; i++)
		{
			img1_filename = dirname + "left0" + to_string(i) + ".jpg";
			img2_filename = dirname + "right0" + to_string(i) + ".jpg";
			img1 = imread(img1_filename);
			img2 = imread(img2_filename);
			if (alg == STEREO_BM)
			{
				img2Gray(img1, img1);
				img2Gray(img2, img2);
			}

			Mat disp8;
			stereoMatch(img1, img2, disp8);

                        showDenseReconstruction(img1, img2, disp8);
		}


		//文件保存
		//if (!disparity_filename.empty())
		//	imwrite(disparity_filename, disp8);
	}







        /****************************************************
                          Stereo calibration
        ****************************************************/
	//displayCorners 表示是否显示每帧标定板的检测结果
	//showRectified 表示是否显示基于双目标定之后的标定图像的矫正
	void stereoCalib(const string dirname, const vector<string>& imagelist, Size boardSize, float squareSize,
		bool displayCorners = false, bool useCalibrated = true, bool showRectified = true)
	{
		if (imagelist.size() % 2 != 0)
		{
			cout << "Error: the image list contains odd (non-even) number of elements\n";
			return;
		}

		/*************************************************
						image pairs detection
		*************************************************/
		const int maxScale = 2;

		// ARRAY AND VECTOR STORAGE:
		vector<vector<Point2f> > imagePoints[2];
		vector<vector<Point3f> > objectPoints;
		Size imageSize;

		int i, j, k, nimages = (int)imagelist.size() / 2;

		imagePoints[0].resize(nimages);
		imagePoints[1].resize(nimages);
		vector<string> goodImageList; //用于存储有效的imagelist

		for (i = j = 0; i < nimages; i++)
		{
			for (k = 0; k < 2; k++)
			{
				const string& filename = imagelist[i * 2 + k];
				Mat img = imread(dirname + filename, 0);
				if (img.empty())
					break;

				if (imageSize == Size())
					imageSize = img.size();
				else if (img.size() != imageSize)
				{
					cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
					break;
				}

				bool found = false;
				vector<Point2f>& corners = imagePoints[k][j];

				//多种scale检测标定板，以防某个scale无法检测
				for (int scale = 1; scale <= maxScale; scale++)
				{
					Mat timg;
					if (scale == 1)
						timg = img;
					else
						resize(img, timg, Size(), scale, scale, INTER_LINEAR_EXACT);
					found = findChessboardCorners(timg, boardSize, corners,
						CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
					if (found)
					{
						if (scale > 1)
						{
							Mat cornersMat(corners);
							cornersMat *= 1. / scale;
						}
						break;
					}
				}

				//显示检测过程imshow
				if (displayCorners)
				{
					cout << filename << endl;
					Mat cimg, cimg1;
					cvtColor(img, cimg, COLOR_GRAY2BGR);
					drawChessboardCorners(cimg, boardSize, corners, found);
					double sf = 640. / MAX(img.rows, img.cols);
					resize(cimg, cimg1, Size(), sf, sf, INTER_LINEAR_EXACT);
					imshow("corners", cimg1);
					char c = (char)waitKey(500);
					if (c == 27 || c == 'q' || c == 'Q') //Allow ESC to quit
						exit(-1);
				}
				else
					putchar('.');


				if (!found)
					break;
				cornerSubPix(img, corners, Size(11, 11), Size(-1, -1),
					TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
						30, 0.01));
			}

			if (k == 2)
			{
				goodImageList.push_back(imagelist[i * 2]);
				goodImageList.push_back(imagelist[i * 2 + 1]);
				j++;
			}
		}
		cout << j << " pairs have been successfully detected.\n";
		nimages = j;
		if (nimages < 2)
		{
			cout << "Error: too little pairs to run the calibration\n";
			return;
		}

		imagePoints[0].resize(nimages);
		imagePoints[1].resize(nimages);
		objectPoints.resize(nimages);

		for (i = 0; i < nimages; i++)
		{
			for (j = 0; j < boardSize.height; j++)
				for (k = 0; k < boardSize.width; k++)
					objectPoints[i].push_back(Point3f(k*squareSize, j*squareSize, 0));
		}



		/*************************************************
						stereo calibration
		*************************************************/
		cout << "Running stereo calibration ...\n";

		Mat cameraMatrix[2], distCoeffs[2];
		cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0], imageSize, 0);
		cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1], imageSize, 0);
		Mat R, T, E, F;

		double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
			cameraMatrix[0], distCoeffs[0],
			cameraMatrix[1], distCoeffs[1],
			imageSize, R, T, E, F,
			CALIB_FIX_ASPECT_RATIO +
			CALIB_ZERO_TANGENT_DIST + //不考虑tangent畸变
			CALIB_USE_INTRINSIC_GUESS +
			CALIB_SAME_FOCAL_LENGTH +
			//CALIB_RATIONAL_MODEL +
			CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
			TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
		cout << "done with RMS error=" << rms << endl;

		// CALIBRATION QUALITY CHECK
		// because the output fundamental matrix implicitly
		// includes all the output information,
		// we can check the quality of calibration using the
		// epipolar geometry constraint: m2^t*F*m1=0
		double err = 0;
		int npoints = 0;
		vector<Vec3f> lines[2];
		for (i = 0; i < nimages; i++)
		{
			int npt = (int)imagePoints[0][i].size();
			Mat imgpt[2];
			for (k = 0; k < 2; k++)
			{
				imgpt[k] = Mat(imagePoints[k][i]);
				undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
				computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);
			}
			for (j = 0; j < npt; j++)
			{
				double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
					imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
					fabs(imagePoints[1][i][j].x*lines[0][j][0] +
						imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
				err += errij;
			}
			npoints += npt;
		}
		cout << "average epipolar err = " << err / npoints << endl;



		M1 = cameraMatrix[0];
		M2 = cameraMatrix[1];
		D1 = distCoeffs[0];
		D2 = distCoeffs[1];


		Mat R1, R2, P1, P2, Q;
		Rect validRoi[2];
		stereoRectify(M1, D1, M2, D2, imageSize, R, T, R1, R2, P1, P2, Q,
			CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
		

                saveParamsToFiles();


		// OpenCV can handle left-right
		// or up-down camera arrangements
		bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

		// COMPUTE AND DISPLAY RECTIFICATION
		if (!showRectified)
			return;

		Mat rmap[2][2];
		// IF BY CALIBRATED (BOUGUET'S METHOD)
		if (useCalibrated)
		{
			// we already computed everything
		}
		// OR ELSE HARTLEY'S METHOD
		else
			// use intrinsic parameters of each camera, but
			// compute the rectification transformation directly
			// from the fundamental matrix
		{
			vector<Point2f> allimgpt[2];
			for (k = 0; k < 2; k++)
				for (i = 0; i < nimages; i++)
					std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));

			F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
			Mat H1, H2;
			stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

			R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
			R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
			P1 = cameraMatrix[0];
			P2 = cameraMatrix[1];
		}

		//Precompute maps for cv::remap()
		initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
		initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

		Mat canvas;
		double sf;
		int w, h;
		if (!isVerticalStereo)
		{
			sf = 600. / MAX(imageSize.width, imageSize.height);
			w = cvRound(imageSize.width*sf);
			h = cvRound(imageSize.height*sf);
			canvas.create(h, w * 2, CV_8UC3);
		}
		else
		{
			sf = 300. / MAX(imageSize.width, imageSize.height);
			w = cvRound(imageSize.width*sf);
			h = cvRound(imageSize.height*sf);
			canvas.create(h * 2, w, CV_8UC3);
		}

		for (i = 0; i < nimages; i++)
		{
			for (k = 0; k < 2; k++)
			{
				Mat img = imread(dirname + goodImageList[i * 2 + k], 0), rimg, cimg;
				remap(img, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);
				cvtColor(rimg, cimg, COLOR_GRAY2BGR);
				Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
				resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
				if (useCalibrated)
				{
					Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
						cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
					rectangle(canvasPart, vroi, Scalar(0, 0, 255), 3, 8);
				}
			}

			if (!isVerticalStereo)
				for (j = 0; j < canvas.rows; j += 16)
					line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
			else
				for (j = 0; j < canvas.cols; j += 16)
					line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
			imshow("rectified", canvas);
			char c = (char)waitKey();
			if (c == 27 || c == 'q' || c == 'Q')
				break;
		}
	}

        void saveParamsToFiles()
	{
		// save intrinsic parameters
		FileStorage fs(intrinsic_filename, FileStorage::WRITE);
		if (fs.isOpened())
		{
			fs << "M1" << M1 << "D1" << D1 << "M2" << M2 << "D2" << D2;
			fs.release();
		}
		else
			cout << "Error: can not save the intrinsic parameters\n";

		fs.open(extrinsic_filename, FileStorage::WRITE);
		if (fs.isOpened())
		{
			fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
			fs.release();
		}
		else
			cout << "Error: can not save the extrinsic parameters\n";

	}

	void stereoCalib(string dirname, string imagelistfn, Size boardSize = Size(9, 6),
		float squareSize = 1.0, bool displayCorners = false,
		bool useCalibrated = true, bool showRectified = false)
	{
		vector<string> imagelist;
		bool ok = readStringList(dirname + imagelistfn, imagelist);
		if (!ok || imagelist.empty())
		{
			cout << "can not open " << imagelistfn << " or the string list is empty" << endl;
			return;
		}

		stereoCalib(dirname, imagelist, boardSize, squareSize, displayCorners,
			useCalibrated, showRectified);
	}


	void stereoCalib(bool displayCorners = false,
		bool useCalibrated = true, bool showRectified = false)
	{
		stereoCalib(dirname, imagelistfn, boardSize);
	}



};



#endif
