双目标定
stereoParams20171130
采集时间：2017.11.30
squareSize = 20 mm
29 images paris
1280*720


/***************************************
			MATLAB标定结果：
***************************************/
1) stereoParams20171130.mat
26/29 images pair (去除了三组效果不好的图) 
重投影误差：0.25 pixels
2) stereoParams20171130_2.mat
29 image pair 
重投影误差：0.27 pixels

t2 = [-62.57,-0.32,-1.31]
K1 = [1308	0	659;
	0	1311	319;
	0	0	1]




/***************************************
			OpenCV标定结果：
***************************************/
stereo_calib.xml, 用于OpenCV双目标定，记录了标定文件列表。

OpenCV标定结果：
29/29检测成功
重投影误差1.10 pixels
intrinsics.yml + extrinsics.yml

t2 = [ -63.06, -0.12, 1.90]
K1 = [ 1375 0., 648;
	0., 1386, 353; 
	0., 0., 1. ]
	
	
/***************************************
			结果分析
***************************************/
照理t2的第一个值应该是正值，但是结果是负值，这应该是和标定板的坐标系有关。
