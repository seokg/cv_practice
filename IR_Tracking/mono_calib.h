#pragma once
#include "use_opencv.h"
//#include "main_init2.h"
#define timeGap 10000000

using namespace cv;
using namespace std;

class Settings
{
public:
	//Settings() : goodInput(false) {}
	enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
};



enum Modes { DETECTING, CAPTURING, CALIBRATING };
Modes mode = DETECTING;
int noOfStereoPairs;
int numimage = 0, cornerImageIndex = 0;
int goIn = 1;
Mat _leftOri, _rightOri;
int64 prevTickCount;
vector<Point2f> cornersLeft, cornersRight;
vector<vector<Point2f> > cameraImagePoints[2];
Size boardsize;

string prefixLeft = "image_";
string prefixRight= ".png";
string postfix;
string dir= "..";

int calibType=1;

bool findChessboardCornersAndDraw(Mat);
void saveImages(Mat, int);
void calibrateStereoCamera(Size);
void calibrateInRealTime(int, int);
void calibrateFromSavedImages(string, string, string, string);


//FINDING CHESSBOARD CORNER AND DRAW
bool findChessboardCornersAndDraw(Mat inputLeft) {
	_leftOri = inputLeft;
	bool foundLeft = false;
	cvtColor(inputLeft, inputLeft, COLOR_BGR2GRAY);
	foundLeft = findChessboardCorners(inputLeft, boardsize, cornersLeft, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
	drawChessboardCorners(_leftOri, boardsize, cornersLeft, foundLeft);
	if (foundLeft){
		return true;
	}
	else {
		return false;
	}
}


//SAVING IMAGE THAT IS CAPTURED
void saveImages(Mat leftImage, int pairIndex) {
	cameraImagePoints[0].push_back(cornersLeft);
	if (calibType == 1) {
		cvtColor(leftImage, leftImage, COLOR_BGR2GRAY);
		std::ostringstream leftString;
		leftString << dir << "/" << prefixLeft << pairIndex << postfix;
		// NEED TO BE FIXED
		//imwrite(leftString.str().c_str(), leftImage);

	}
}


//BOARD CORNER POSTION 
static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
	Settings::Pattern patternType)
{
	corners.clear();


	switch (patternType)
	{
	case Settings::CHESSBOARD:

	case Settings::CIRCLES_GRID:
		for (int i = 0; i < boardSize.height; ++i)
		for (int j = 0; j < boardSize.width; ++j)
			corners.push_back(Point3f(j*squareSize, i*squareSize, 0));
		break;

	case Settings::ASYMMETRIC_CIRCLES_GRID:
		for (int i = 0; i < boardSize.height; i++)
		for (int j = 0; j < boardSize.width; j++)
			corners.push_back(Point3f((2 * j + i % 2)*squareSize, i*squareSize, 0));
		break;
	default:
		break;
	}
}



// CALIBRATION
static bool runCalibrealtime( Size& boardSize, float squareSize, 
	Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
	vector<vector<Point2f> > imagePoints, 
	vector<Mat>& rvecs, vector<Mat>& tvecs,
	vector<float>& reprojErrs, double& totalAvgErr)
{
	int flag = 0;
	flag |= CALIB_FIX_PRINCIPAL_POINT;
	flag |= CALIB_ZERO_TANGENT_DIST;
	flag |= CALIB_FIX_ASPECT_RATIO;
	//! [fixed_aspect]
	cameraMatrix = Mat::eye(3, 3, CV_64F);
	if (flag & CALIB_FIX_ASPECT_RATIO)
		cameraMatrix.at<double>(0, 0) = 1;
	//! [fixed_aspect]
	distCoeffs = Mat::zeros(8, 1, CV_64F);



	vector<vector<Point3f> > objectPoints(1);
	calcBoardCornerPositions(boardSize, squareSize, objectPoints[0], Settings::CHESSBOARD);

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	//Find intrinsic and extrinsic camera parameters
	double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
		distCoeffs, rvecs, tvecs, flag | CALIB_FIX_K4 | CALIB_FIX_K5);

	cout << "Re-projection error reported by calibrateCamera: " << rms << endl;
	
	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

	//totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
	//                                         rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

	return ok;
}




int frame_captured = 1;

bool calibrateRealTime(
	VideoCapture video, Mat& frame,
	//int cam1, 
	Size& boardSize, float& squareSize, Mat& cameraMatrix, Mat& distCoeffs,
	vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
	vector<float>& reprojErrs, double& totalAvgErr)
{
	//VideoCapture maincam(cam1);
	//if (!maincam.isOpened()){
	//	cout << "Error:unable to load the camera" << endl;
	//	exit(-1);
	//}
	//Mat inputcamera;


	Mat bufinputcamera;
	bool foundCorners = false;
	vector<Point2f> pointBuf;
	int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;// | CALIB_CB_FAST_CHECK;
	//int i = 1;
	video >> frame;
	//maincam >> inputcamera;
	bufinputcamera = frame;

	//rt_frame.copyTo(bufinputcamera);
	foundCorners = findChessboardCorners(frame, boardSize, pointBuf, chessBoardFlags);
	if (foundCorners){
		Mat viewGray;
		cvtColor(frame, viewGray, COLOR_BGR2GRAY);
		cornerSubPix(viewGray, pointBuf, Size(11, 11),
			Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));

		mode = CAPTURING;
		if (foundCorners && mode == CAPTURING && (numimage < 7)) {
			imagePoints.push_back(pointBuf);
			int64 thisTick = getTickCount();
			int64 diff = thisTick - prevTickCount;
			if (goIn == 1 || diff >= timeGap) {

				goIn = 0;
				saveImages(bufinputcamera, ++numimage);
				prevTickCount = getTickCount();
				cout << "image captured " << frame_captured << endl;
				frame_captured++;
				//std::ostringstream imageIndex;
				//imageIndex << frame_captured << "/" << noOfStereoPairs;
				//putText(frame, imageIndex.str().c_str(), Point(50, 70), FONT_HERSHEY_PLAIN, 0.9, Scalar(0, 255, 255), 2);
				if (numimage == 7){
					mode = CALIBRATING;
				}
			}
		}

		drawChessboardCorners(frame, boardSize, pointBuf, foundCorners);
	}

	// CAPTURED IMAGE NUMBER
	std::ostringstream imageIndex;
	imageIndex << (frame_captured - 1) << "/" << 7;
	putText(frame, imageIndex.str().c_str(), Point(50, 70), FONT_HERSHEY_PLAIN, 2, Scalar(0, 255, 255), 2);
	
	// GUIDE TO ROTATE
	std::ostringstream guide;
	guide << "please rotate the camera";
	putText(frame, guide.str().c_str(), Point(50, 100), FONT_HERSHEY_PLAIN, 2, Scalar(0, 255, 255), 2);

	//imshow("real time video", frame);
	Size imageSize = frame.size();
	if (mode == CALIBRATING) {
		runCalibrealtime(boardSize, squareSize, imageSize, cameraMatrix, distCoeffs,
			imagePoints, rvecs, tvecs,
			reprojErrs, totalAvgErr);
		//calibrateStereoCamera(inputLeft.size());
		//cout << "Calibration Complete" << endl;
		std::ostringstream stringcomplete;
		stringcomplete << "Calibration Completed";
		numimage = 0; frame_captured = 1;
		
		putText(frame, stringcomplete.str().c_str(), Point(100, 150), FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 255), 2);
		//waitKey();
		return  true;

	}

	//char keyBoardInput = (char)waitKey(10);
	//if (keyBoardInput == 'q' || keyBoardInput == 'Q') {
	//	//cout << "Exiting Calibration Program" << endl;
	//	exit(-1);
	//}
	//else if (keyBoardInput == 'c' || keyBoardInput == 'C') {
	//	//cout << "Capturing starts" << endl;
	//	mode = CAPTURING;
	//	//putText(frame, "CAPTURING", Point(50, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);
	//}
	//else if (keyBoardInput == 'p' || keyBoardInput == 'P') {
	//	//cout << "Calibration starts" << endl;
	//	mode = CALIBRATING;
	//	//putText(frame, "CALIBRATING", Point(50, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);
	//}
	
	return false;
}