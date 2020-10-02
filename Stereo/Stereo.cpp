// Stereo.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>

#include "StereoReconstructor.h"


void test()
{
	StereoReconstructor stereor;
	//TODO 2020.10.2 路径以修改，需要重新调整
	stereor.dirname = "C:/Users/w9349/OneDrive - Platinum/mycodelib/stereoCalibration/stereo calibration20171130/";

	//双目重建测试
	stereor.alg = StereoReconstructor::STEREO_BM;
	stereor.img1_filename = stereor.dirname + "left/left01.png";
	stereor.img2_filename = stereor.dirname + "right/right01.png";
	stereor.testSparse();


	//双目标定测试
	//stereor.imagelistfn = "stereo_calib.xml";
	//stereor.boardSize = Size(8, 5);
	//stereor.squareSize = 20.0;
	//stereor.stereoCalib(false, true, false);
}






int main()
{
    std::cout << "Hello World!\n";
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
