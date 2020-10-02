%stereo test
dirname = 'C:/Users/w9349/OneDrive - Platinum/mycodelib/stereoCalibration/stereo calibration20171130/';
imgl = imread([dirname, 'left/left01.png']);
imgr = imread([dirname, 'right/right01.png']);
imgl = rgb2gray(imgl);
imgr = rgb2gray(imgr);

% filter 之后，重建效果更好
imgl = imgaussfilt(imgl,3);
imgr = imgaussfilt(imgr,3);

%%  1 稠密重建
% rectify
[J1,J2] = rectifyStereoImages(imgl,imgr,stereoParams);
figure
imshowpair(J1,J2,'montage')

%disparityMap: Disparity map between stereo images
% 2019 版本 --> disparitySGM
disparityMap = disparity(J1, J2);
figure
imshow(disparityMap,[0,64],'InitialMagnification',50);

%reconstructScene: Reconstruct 3-D scene from disparity map
pointCloud = reconstructScene(disparityMap,stereoParams);
figure,
pcshow(pointCloud);

%% 1.1 注意xy坐标和行列坐标相反
format bank
% x,y 坐标
point = [400,300];
testpoint = pointCloud(point(2),point(1),:);
testpoint = (testpoint(:))';
disp(point), disp(testpoint)

point = [500,500];
testpoint = pointCloud(point(2),point(1),:);
testpoint = (testpoint(:))';
disp(point), disp(testpoint)

point = [900,200];
testpoint = pointCloud(point(2),point(1),:);
testpoint = (testpoint(:))';
disp(point), disp(testpoint)

%sample：标定板左上角点
%可以看出，这个结果与稀疏重建相差甚远，可能是标定板重建效果不好导致的。
point = [313,289];
testpoint = pointCloud(point(2),point(1),:);
testpoint = (testpoint(:))';
disp(point), disp(testpoint)
% 结果显示表明，坐标系是没有问题的，符合坐标系的定义

%% 2 稀疏重建
%注意matchedpoint不是rectify之后的坐标，而是原始的
figure,imshow(imgl)
figure,imshow(imgr)
%M-by-2 matrix of M number of [x y] coordinates
%sample：标定板左上角点
matchedPoints1 = [401,327];
matchedPoints2 = [183,304];
worldPoints = triangulate (matchedPoints1, matchedPoints2, stereoParams)

    
    