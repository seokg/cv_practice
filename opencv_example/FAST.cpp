// ************** //
// FAST Algorithm //
// ************** //


//#include "cv.h"
//#include "cvaux.hpp"
//#include "highgui.h"
#include "use_opencv.h"
#include <time.h>
#include <iostream>

using namespace std;
using namespace cv;

double diffClock(clock_t begin, clock_t end);



const char* source_window = "Image";

int main(int argc, char** argv)
{

	bool run = true;


	Mat input = imread("HMD.png", 0);

	int key = 0;


	while (run)
	{
		clock_t begin, end;
		key = cvWaitKey(10);

		imshow(source_window, input);

		int threshold = 9;
		vector<KeyPoint> keypoints;

		if (key == 'a'){

			Mat mat;
			FAST(input, keypoints, threshold, true);
			Mat output;
			drawKeypoints(input, keypoints, output);
			imwrite("sift_result.jpg", output);

			imshow("result", output);

		}
		else if (key == 'x'){
			run = false;
		}
	}
	cvDestroyWindow("stream");
	return 0;
}
