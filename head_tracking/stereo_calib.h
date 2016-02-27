
#include "use_opencv.h"

using namespace cv;
using namespace std;



static int print_help()
{
	cout <<
		" Given a list of chessboard images, the number of corners (nx, ny)\n"
		" on the chessboards, and a flag: useCalibrated for \n"
		"   calibrated (0) or\n"
		"   uncalibrated \n"
		"     (1: use cvStereoCalibrate(), 2: compute fundamental\n"
		"         matrix separately) stereo. \n"
		" Calibrate the cameras and display the\n"
		" rectified results along with the computed disparity images.   \n" << endl;
	cout << "Usage:\n ./stereo_calib -w board_width -h board_height [-nr /dot not view results/] <image list XML/YML file>\n" << endl;
	return 0;
}



static void StereoCalib_camera_parameter(
	InputOutputArray R, InputOutputArray T,
	InputOutputArray cameraMatrix_ref, InputOutputArray distCoeffs_ref,
	InputOutputArray cameraMatrix_pair, InputOutputArray distCoeffs_pair,
	const vector<string>& imagelist, Size boardSize,
	bool useCalibrated = true, bool showRectified = true 
	)
	{
	//R = Mat::zeros(3, 3, CV_64F);
	//T = Mat::zeros(3, 1, CV_64F);
	//cameraMatrix_ref = Mat::eye(3, 3, CV_64F);
	//distCoeffs_ref = Mat::zeros(5, 1, CV_64F);	
	//cameraMatrix_pair = Mat::eye(3, 3, CV_64F);
	//distCoeffs_pair = Mat::zeros(5, 1, CV_64F);



	if (imagelist.size() % 2 != 0)
	{
		cout << "Error: the image list contains odd (non-even) number of elements\n";
		return;
	}

	bool displayCorners = false;//true;
	const int maxScale = 1;//2;
	const float squareSize = 10;//1.f;  // Set this to your actual square size
	// ARRAY AND VECTOR STORAGE:

	vector<vector<Point2f> > imagePoints[2];
	vector<vector<Point3f> > objectPoints;
	Size imageSize;

	int i, j, k, nimages = (int)imagelist.size() / 2;

	imagePoints[0].resize(nimages);
	imagePoints[1].resize(nimages);
	vector<string> goodImageList;

	for (i = j = 0; i < nimages; i++)
	{
		for (k = 0; k < 2; k++)
		{
			const string& filename = imagelist[i * 2 + k];
			Mat img = imread(filename, 0);
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
			for (int scale = 1; scale <= maxScale; scale++)
			{
				Mat timg;
				if (scale == 1)
					timg = img;
				else
					resize(img, timg, Size(), scale, scale);
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


			if (!found)
				break;

			cornerSubPix(img, corners, Size(5, 5), Size(-1, -1),
				TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
				30, 0.01));
			if (!displayCorners)//(displayCorners)
			{
				cout << filename << endl;
				Mat cimg, cimg1;
				cvtColor(img, cimg, COLOR_GRAY2BGR);
				drawChessboardCorners(cimg, boardSize, corners, found);
				double sf = 640. / MAX(img.rows, img.cols);
				resize(cimg, cimg1, Size(), sf, sf);
				imshow("corners", cimg1);
				char c = (char)waitKey(500);
				if (c == 27 || c == 'q' || c == 'Q') //Allow ESC to quit
					exit(-1);
			}
			else
				cout << "image:" << i << "\n";


		}
		if (k == 2)
		{
			goodImageList.push_back(imagelist[i * 2]);
			goodImageList.push_back(imagelist[i * 2 + 1]);
			j++;
			cout << "image:" << i << "\n";
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

	cout << "Running stereo calibration ...\n";

	//distCoeffs[2];
	//cameraMatrix_ref = Mat::eye(3, 3, CV_64F);
	//cameraMatrix_pair = Mat::eye(3, 3, CV_64F);


	//Mat R, T,
	Mat E, F;
	
	double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
		cameraMatrix_ref, distCoeffs_ref,
		cameraMatrix_pair, distCoeffs_pair,
		imageSize, R, T, E, F,
		CALIB_FIX_ASPECT_RATIO |
		CALIB_ZERO_TANGENT_DIST |
		CALIB_SAME_FOCAL_LENGTH |
		//CALIB_RATIONAL_MODEL |
		CALIB_FIX_K3 | CALIB_FIX_K4 | CALIB_FIX_K5,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));


	cout << "done with RMS error=" << rms << endl;



	// 
	// NEED TO THINK ABOUT
	//
	//


	//// CALIBRATION QUALITY CHECK
	//// because the output fundamental matrix implicitly
	//// includes all the output information,
	//// we can check the quality of calibration using the
	//// epipolar geometry constraint: m2^t*F*m1=0
	//double err = 0;
	//int npoints = 0;
	//vector<Vec3f> lines[2];
	//for (i = 0; i < nimages; i++)
	//{
	//	int npt = (int)imagePoints[0][i].size();
	//	Mat imgpt[2];
	//	for (k = 0; k < 2; k++)
	//	{
	//		imgpt[k] = Mat(imagePoints[k][i]);
	//		undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
	//		computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);
	//	}
	//	for (j = 0; j < npt; j++)
	//	{
	//		double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
	//			imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
	//			fabs(imagePoints[1][i][j].x*lines[0][j][0] +
	//			imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
	//		err += errij;
	//	}
	//	npoints += npt;
	//}
	//cout << "average reprojection err = " << err / npoints << endl;

	///***********************************************
	//<saving parameter>
	//**********************************************/
	//// save intrinsic parameters
	//FileStorage fs("../head_tracking/images/intrinsics.yml", FileStorage::WRITE);
	////FileStorage fs("../head_tracking/images_sample/intrinsics.yml", FileStorage::WRITE); // sample data from opencv
	//if (fs.isOpened())
	//{
	//	fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
	//		"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
	//	fs.release();
	//}
	//else
	//	cout << "Error: can not save the intrinsic parameters\n";

	//Mat R1, R2, P1, P2, Q;
	//Rect validRoi[2];

	//stereoRectify(cameraMatrix[0], distCoeffs[0],
	//	cameraMatrix[1], distCoeffs[1],
	//	imageSize, R, T, R1, R2, P1, P2, Q,
	//	CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
	////saving extrinsic parameters
	//fs.open("../head_tracking/images/extrinsics.yml", FileStorage::WRITE);
	////fs.open("../head_tracking/images_sample/extrinsics.yml", FileStorage::WRITE); // sample data from opencv
	//if (fs.isOpened())
	//{
	//	fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
	//	fs.release();
	//}
	//else
	//	cout << "Error: can not save the extrinsic parameters\n";
	///***********************************************
	//</saving parameter>
	//**********************************************/




	//// OpenCV can handle left-right
	//// or up-down camera arrangements
	//bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

	//// COMPUTE AND DISPLAY RECTIFICATION
	//if (!showRectified)
	//	return;

	//Mat rmap[2][2];
	//// IF BY CALIBRATED (BOUGUET'S METHOD)
	//if (useCalibrated)
	//{
	//	// we already computed everything
	//}
	//// OR ELSE HARTLEY'S METHOD
	//else
	//	// use intrinsic parameters of each camera, but
	//	// compute the rectification transformation directly
	//	// from the fundamental matrix
	//{
	//	vector<Point2f> allimgpt[2];
	//	for (k = 0; k < 2; k++)
	//	{
	//		for (i = 0; i < nimages; i++)
	//			std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
	//	}
	//	F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
	//	Mat H1, H2;
	//	stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

	//	R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
	//	R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
	//	P1 = cameraMatrix[0];
	//	P2 = cameraMatrix[1];
	//}

	////Precompute maps for cv::remap()

	////	fisheye::initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	////	fisheye::initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

	//initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	//initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

	//Mat canvas;
	//double sf;
	//int w, h;
	//if (!isVerticalStereo)
	//{
	//	sf = 600. / MAX(imageSize.width, imageSize.height);
	//	w = cvRound(imageSize.width*sf);
	//	h = cvRound(imageSize.height*sf);
	//	canvas.create(h, w * 2, CV_8UC3);
	//}
	//else
	//{
	//	sf = 600. / MAX(imageSize.width, imageSize.height);
	//	w = cvRound(imageSize.width*sf);
	//	h = cvRound(imageSize.height*sf);
	//	canvas.create(h * 2, w, CV_8UC3);
	//}

	//for (i = 0; i < nimages; i++)
	//{
	//	for (k = 0; k < 2; k++)
	//	{
	//		Mat img = imread(goodImageList[i * 2 + k], 0), rimg, cimg;
	//		remap(img, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);
	//		cvtColor(rimg, cimg, COLOR_GRAY2BGR);
	//		Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
	//		resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
	//		if (useCalibrated)
	//		{
	//			Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
	//				cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
	//			rectangle(canvasPart, vroi, Scalar(0, 0, 255), 3, 8);
	//		}
	//	}

	//	if (!isVerticalStereo)
	//	for (j = 0; j < canvas.rows; j += 16)
	//		line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
	//	else
	//	for (j = 0; j < canvas.cols; j += 16)
	//		line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
	//	imshow("rectified", canvas);
	//	char c = (char)waitKey();
	//	if (c == 27 || c == 'q' || c == 'Q')
	//		break;
	//}



}

static void StereoCalib(const vector<string>& imagelist, Size boardSize, 
	bool useCalibrated = true, bool showRectified = true)
{
	if (imagelist.size() % 2 != 0)
	{
		cout << "Error: the image list contains odd (non-even) number of elements\n";
		return;
	}

	bool displayCorners = false;//true;
	const int maxScale = 1;//2;
	const float squareSize = 10;//1.f;  // Set this to your actual square size
	// ARRAY AND VECTOR STORAGE:

	vector<vector<Point2f> > imagePoints[2];
	vector<vector<Point3f> > objectPoints;
	Size imageSize;

	int i, j, k, nimages = (int)imagelist.size() / 2;

	imagePoints[0].resize(nimages);
	imagePoints[1].resize(nimages);
	vector<string> goodImageList;

	for (i = j = 0; i < nimages; i++)
	{
		for (k = 0; k < 2; k++)
		{
			const string& filename = imagelist[i * 2 + k];
			Mat img = imread(filename, 0);
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
			for (int scale = 1; scale <= maxScale; scale++)
			{
				Mat timg;
				if (scale == 1)
					timg = img;
				else
					resize(img, timg, Size(), scale, scale);
				// resize (input:source, output:destination, output size, scale factore:fx, scale factor:fy, interpolation method)
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


			if (!found)
				break;


			//			cornerSubPix(img, corners, Size(11, 11), Size(-1, -1),
			//			TermCriteria(TermCriteria::COUNT+TermCriteria::EPS,
			//			30, 0.01));
			cornerSubPix(img, corners, Size(5, 5), Size(-1, -1),
				TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
				30, 0.01));

			//for (size_t i = 0; i < corners.size(); i++)
			//{
			//	cout << " -- Refined Corner [" << i << "]  (" << corners[i].x << "," << corners[i].y << ")" << endl;
			//}

			//displaying chessboard corner
			if (!displayCorners)//(displayCorners)
			{
				cout << filename << endl;
				Mat cimg, cimg1;
				cvtColor(img, cimg, COLOR_GRAY2BGR);
				drawChessboardCorners(cimg, boardSize, corners, found);
				double sf = 640. / MAX(img.rows, img.cols);
				resize(cimg, cimg1, Size(), sf, sf);
				imshow("corners", cimg1);
				char c = (char)waitKey(500);
				if (c == 27 || c == 'q' || c == 'Q') //Allow ESC to quit
					exit(-1);
			}
			else
				cout << "image:" << i << "\n";

			// ********* //
			// MOVED UP  //
			// ********* //
			//			if (!found)
			//				break;
			//			cornerSubPix(img, corners, Size(5, 5), Size(-1, -1),
			//				TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
			//				50, 0.001));
			//
			//			for (size_t i = 0; i < corners.size(); i++)
			//			{
			//				cout << " -- Refined Corner [" << i << "]  (" << corners[i].x << "," << corners[i].y << ")" << endl;
			//			}


		}
		if (k == 2)
		{
			goodImageList.push_back(imagelist[i * 2]);
			goodImageList.push_back(imagelist[i * 2 + 1]);
			j++;
			cout << "image:" << i << "\n";
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

	cout << "Running stereo calibration ...\n";

	Mat cameraMatrix[2], distCoeffs[2];
	cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
	cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
	Mat R, T, E, F;
	//double stereoCalibrate(InputArrayOfArrays objectPoints, InputArrayOfArrays imagePoints1, InputArrayOfArrays imagePoints2, 
	//InputOutputArray cameraMatrix1, InputOutputArray distCoeffs1, InputOutputArray cameraMatrix2, InputOutputArray distCoeffs2, 
	//Size imageSize, OutputArray R, OutputArray T, OutputArray E, OutputArray F, TermCriteria criteria=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 1e-6),
	//int flags=CALIB_FIX_INTRINSIC )
	/*  double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
	cameraMatrix[0], distCoeffs[0],
	cameraMatrix[1], distCoeffs[1],
	imageSize, R, T, E, F,
	CALIB_FIX_ASPECT_RATIO +
	CALIB_ZERO_TANGENT_DIST +
	CALIB_SAME_FOCAL_LENGTH +
	CALIB_RATIONAL_MODEL +
	CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
	TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 100, 1e-5) );
	*/
	double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
		cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, E, F,
		CALIB_FIX_ASPECT_RATIO |
		CALIB_ZERO_TANGENT_DIST |
		CALIB_SAME_FOCAL_LENGTH |
		//CALIB_RATIONAL_MODEL |
		CALIB_FIX_K3 | CALIB_FIX_K4 | CALIB_FIX_K5,
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
	cout << "average reprojection err = " << err / npoints << endl;

	/***********************************************
	<saving parameter>
	**********************************************/
	// save intrinsic parameters
	FileStorage fs("../head_tracking/images/intrinsics.yml", FileStorage::WRITE);
	//FileStorage fs("../head_tracking/images_sample/intrinsics.yml", FileStorage::WRITE); // sample data from opencv
	if (fs.isOpened())
	{
		fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
			"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
		fs.release();
	}
	else
		cout << "Error: can not save the intrinsic parameters\n";

	Mat R1, R2, P1, P2, Q;
	Rect validRoi[2];

	stereoRectify(cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, R1, R2, P1, P2, Q,
		CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
	//saving extrinsic parameters
	fs.open("../head_tracking/images/extrinsics.yml", FileStorage::WRITE);
	//fs.open("../head_tracking/images_sample/extrinsics.yml", FileStorage::WRITE); // sample data from opencv
	if (fs.isOpened())
	{
		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs.release();
	}
	else
		cout << "Error: can not save the extrinsic parameters\n";
	/***********************************************
	</saving parameter>
	**********************************************/




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
		{
			for (i = 0; i < nimages; i++)
				std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
		}
		F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
		Mat H1, H2;
		stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

		R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
		R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
		P1 = cameraMatrix[0];
		P2 = cameraMatrix[1];
	}

	//Precompute maps for cv::remap()

	//	fisheye::initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	//	fisheye::initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

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
		sf = 600. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h * 2, w, CV_8UC3);
	}

	for (i = 0; i < nimages; i++)
	{
		for (k = 0; k < 2; k++)
		{
			Mat img = imread(goodImageList[i * 2 + k], 0), rimg, cimg;
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


static bool readStringList(const string& filename, vector<string>& l)
{
	l.resize(0);
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	FileNode n = fs.getFirstTopLevelNode();
	if (n.type() != FileNode::SEQ)
		return false;
	FileNodeIterator it = n.begin(), it_end = n.end();
	for (; it != it_end; ++it)
		l.push_back((string)*it);
	return true;
}


bool cv::findChessboardCorners(InputArray _image, Size patternSize,
	OutputArray corners, int flags)
{
	int count = patternSize.area() * 2;
	std::vector<Point2f> tmpcorners(count + 1);
	Mat image = _image.getMat(); CvMat c_image = image;
	bool ok = cvFindChessboardCorners(&c_image, patternSize,
		(CvPoint2D32f*)&tmpcorners[0], &count, flags) > 0;
	if (count > 0)
	{
		tmpcorners.resize(count);
		Mat(tmpcorners).copyTo(corners);
	}
	else
		corners.release();
	return ok;
}
