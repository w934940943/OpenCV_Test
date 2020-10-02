//Example 9-2. Toy program for using a mouse to draw boxes on the screen
#include <opencv2/opencv.hpp>
using namespace cv;
// Define our callback which we will install for mouse events
void my_mouse_callback(
	int event, int x, int y, int flags, void* param
);
Rect box;
bool drawing_box = false;

// A little subroutine to draw a box onto an image
void draw_box(cv::Mat& img, cv::Rect box) {
	cv::rectangle(
		img,
		box,
		cv::Scalar(0x00, 0x00, 0xff) /* red */
	);
}

int main_mouse(int argc, char** argv)
{
	cv::Mat image = Mat::zeros(200, 200, CV_8UC3);
	Mat temp;
	box = cv::Rect(-1, -1, 0, 0);
	cv::namedWindow("Box Example");

	// Here is the crucial moment where we actually install the callback. 
	// Note that we set the value of 'params' to be the image we are working 
	// with so that the callback will have the image to edit.
	cv::setMouseCallback(
		"Box Example",
		my_mouse_callback,
		(void*)&image
	);

	// The main program loop. Here we copy the working image
	// to the temp image, and if the user is drawing, then
	// put the currently contemplated box onto that temp image.
	// Display the temp image, and wait 15ms for a keystroke,
	// then repeat.
	imshow("Box Example", image);
	waitKey();
	/*
	for (;;) {
		image.copyTo(temp);
		if (drawing_box) draw_box(temp, box);
		cv::imshow("Box Example", temp);
		if (cv::waitKey(15) == 27) break;
	}*/
	return 0;
}

// This is our mouse callback. If the user
// presses the left button, we start a box.
// When the user releases that button, then we
// add the box to the current image. When the
// mouse is dragged (with the button down) we
// resize the box.
void my_mouse_callback(
	int event, int x, int y, int flags, void* param
) {
	cv::Mat& image = *(cv::Mat*) param;
	switch (event)
	{
	case cv::EVENT_MOUSEMOVE:
	{
		if (drawing_box)
		{
			box.width = x - box.x;
			box.height = y - box.y;
		}
	}
	break;
	case cv::EVENT_LBUTTONDOWN:
	{
		drawing_box = true;
		box = cv::Rect(x, y, 0, 0);
	}
	break;
	case cv::EVENT_LBUTTONUP:
	{
		drawing_box = false;
		if (box.width < 0)
		{
			box.x += box.width;
			box.width *= -1;
		}
		if (box.height < 0)
		{
			box.y += box.height;
			box.height *= -1;
		}
		draw_box(image, box);
		imshow("Box Example", image);
	}
	break;
	}
}