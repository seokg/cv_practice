/* **************************************************

HEAD TRACKING USING STEREO CAMERA

edited by seo.kg
Jan.06.2016~

************************************************** */



#include "use_opencv.h"
#include "stereo_calib.h"

using namespace cv;
using namespace std;



int main(int argc, char** argv)
{



	// *************************************************************** //
	// Stereo Calib                                                    //
	// TODO                                                            //
	// 1. instead of image list get the image from the camera directly //
	//    left and right. aruond 8 image each for good detection.      //
	// 2. extract the camera parameter (both in/extrinsic) and save.   //
	//                                                                 //
	// *************************************************************** //
	Size boardSize = Size(9, 11);
	bool showRectified = true;
	vector<string> imagelist;
	Mat R, T, cameraMatrix_ref, cameraMatrix_pair, distCoeffs_ref, distCoeffs_pair;

	R = Mat::zeros(3, 3, CV_64F);
	T = Mat::zeros(3, 1, CV_64F);
	cameraMatrix_ref = Mat::eye(3, 3, CV_64F);
	distCoeffs_ref = Mat::zeros(5, 1, CV_64F);
	cameraMatrix_pair = Mat::eye(3, 3, CV_64F);
	distCoeffs_pair = Mat::zeros(5, 1, CV_64F);

	string imagelistfn = "../head_tracking/images/data.xml";
	readStringList(imagelistfn, imagelist);
	StereoCalib_camera_parameter(R, T,
		cameraMatrix_ref,  distCoeffs_ref,
		cameraMatrix_pair,  distCoeffs_pair,
		imagelist, boardSize, true, showRectified);





		
	// *************************************************************** // 
	// Real Time Video for calculating pose                            //
	// TODO                                                            //
	// 1. get real time video input                                    //
	// 2. save the prev frame                                          //
	// 3. compare the fame for pose estimation                         //
	// 4. output the tracked data                                      //
	// *************************************************************** //

	// *********************************** //
	// Capturing the image from the camera //
	// *********************************** //
	VideoCapture cap(0); // open the default camera
	if (!cap.isOpened())  // check if we succeeded
		return -1;

	Mat output;
	namedWindow("agast_result", 1);
	for (;;) // continously capture the data
	{
		Mat img;
		Mat current_frame;
		Mat prev_frame;
		cap >> current_frame; // get a new frame from camera
		cvtColor(current_frame, img, CV_BGR2GRAY);

		// ***** //
		// SIFT  //
		// TODO: need to find the a lib, make my own, or use repository
		// ***** //
		int threshold = 50;
		vector<KeyPoint> keypoints;
		AGAST(img, keypoints, threshold, true);
		// drawing the restult
		drawKeypoints(img, keypoints, output);
		imshow("agast_result", img);
		// saving the current frame
		prev_frame = prev_frame;
		// saving the current keypoints
		vector<KeyPoint> prev_keypoints = keypoints;
	
		// **************** //
		// Feature Matching //
		// **************** //
		
		if (waitKey(30) >= 0) break;
	}

	return 0;
}





// edit needed 
// to make array of 3D points arrays for each image
// 

typedef struct{
	vector<KeyPoint> keypoints;
}features;

typedef struct{
	vector<features> list;
}image_list_feat;



//************************* //
//stereo camera calibration //
//************************* //
// ************************* //
// image extraction from xml //
// ************************* //
/*	imagelistfn = "../head_tracking/oculus_rift/marker/data.xml";
vector<string> imagelist;
bool ok = readStringList(imagelistfn, imagelist);

if (!ok || imagelist.empty())
{
cout << "can not open " << imagelistfn << " or the string list is empty" << endl;
return print_help();
}
int nimages = (int)imagelist.size();
*/


// ************************** //Q
// AGAST: feature point match //
// ************************** //
/*	int threshold = 50;
bool run = true;
int key = 0;

for (int i = 0; i < nimages; i++){
const string& filename = imagelist[i];
Mat img = imread(filename, 0);
if (img.empty())
break;
key = cvWaitKey(10);
//imshow("Image", img);
int threshold = 50;
vector<KeyPoint> keypoints;
AGAST(img, keypoints, threshold, true);
// drawing the restult
Mat output;
drawKeypoints(img, keypoints, output);
imshow("agast_result", output);
}
*/




/*
int type;
IplImage* color_img;
CvCapture* cv_cap = cvCaptureFromCAM(0);
cvNamedWindow("Video", 0); // create window
for (;;) {
	color_img = cvQueryFrame(cv_cap); // get frame
	if (color_img != 0)
		cvShowImage("Video", color_img); // show frame
	type = cvWaitKey(10); // wait 10 ms or for key stroke
	if (type == 27)
		break; // if ESC, break and quit
}
// clean up
cvReleaseCapture(&cv_cap);
cvDestroyWindow("Video");
*/




/* This is sample from the OpenCV book. The copyright notice is below */

/* *************** License:**************************
Oct. 3, 2008
Right to use this code in any way you want without warranty, support or any guarantee of it working.

BOOK: It would be nice if you cited it:
Learning OpenCV: Computer Vision with the OpenCV Library
by Gary Bradski and Adrian Kaehler
Published by O'Reilly Media, October 3, 2008

AVAILABLE AT:
http://www.amazon.com/Learning-OpenCV-Computer-Vision-Library/dp/0596516134
Or: http://oreilly.com/catalog/9780596516130/
ISBN-10: 0596516134 or: ISBN-13: 978-0596516130

OPENCV WEBSITES:
Homepage:      http://opencv.org
Online docs:   http://docs.opencv.org
Q&A forum:     http://answers.opencv.org
Issue tracker: http://code.opencv.org
GitHub:        https://github.com/Itseez/opencv/
************************************************** */
