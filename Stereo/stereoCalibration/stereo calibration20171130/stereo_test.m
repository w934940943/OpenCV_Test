%stereo test
dirname = 'C:/Users/w9349/OneDrive - Platinum/mycodelib/stereoCalibration/stereo calibration20171130/';
imgl = imread([dirname, 'left/left01.png']);
imgr = imread([dirname, 'right/right01.png']);
imgl = rgb2gray(imgl);
imgr = rgb2gray(imgr);

% filter ֮���ؽ�Ч������
imgl = imgaussfilt(imgl,3);
imgr = imgaussfilt(imgr,3);

%%  1 �����ؽ�
% rectify
[J1,J2] = rectifyStereoImages(imgl,imgr,stereoParams);
figure
imshowpair(J1,J2,'montage')

%disparityMap: Disparity map between stereo images
% 2019 �汾 --> disparitySGM
disparityMap = disparity(J1, J2);
figure
imshow(disparityMap,[0,64],'InitialMagnification',50);

%reconstructScene: Reconstruct 3-D scene from disparity map
pointCloud = reconstructScene(disparityMap,stereoParams);
figure,
pcshow(pointCloud);

%% 1.1 ע��xy��������������෴
format bank
% x,y ����
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

%sample���궨�����Ͻǵ�
%���Կ�������������ϡ���ؽ������Զ�������Ǳ궨���ؽ�Ч�����õ��µġ�
point = [313,289];
testpoint = pointCloud(point(2),point(1),:);
testpoint = (testpoint(:))';
disp(point), disp(testpoint)
% �����ʾ����������ϵ��û������ģ���������ϵ�Ķ���

%% 2 ϡ���ؽ�
%ע��matchedpoint����rectify֮������꣬����ԭʼ��
figure,imshow(imgl)
figure,imshow(imgr)
%M-by-2 matrix of M number of [x y] coordinates
%sample���궨�����Ͻǵ�
matchedPoints1 = [401,327];
matchedPoints2 = [183,304];
worldPoints = triangulate (matchedPoints1, matchedPoints2, stereoParams)

    
    