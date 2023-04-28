#include <opencv2/opencv.hpp>

#include <iostream>

using namespace cv;

int main()
{
	VideoCapture cap(10);
	while (cap.isOpened())
	{
		Mat img1,img2,img3,img4;
		cap >> img1;
		imshow("Frame", img1);
		char c = (char)waitKey(10);
		if (c == 'q')
			break;
	}
	cap.release();
	destroyAllWindows();
	return 0;
}
