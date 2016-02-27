#include "main_init.h"

int main(int argc, char** argv) {
	int flag = 0;
	bool KF_flag = false;
	int count = 0;
	// *************************************************************** //
	// Mono Calibration                                                //
	// TODO                                                            //
	// *************************************************************** //

	//calibrateRealTime(0, boardSize, squareSize, cameraMatrix, distCoeffs,
	//		imagePoints, rvecs, tvecs,
	//		reprojErrs, totalAvgErr);

	cameraMatrix = (Mat_<float>(3, 3) << 889.651, 0, 319.5, 0, 889.651, 239.5, 0, 0, 1);
	distCoeffs = (Mat_<float>(5, 1) << -0.06471, 0.4719, 0, 0, -2.24498);//distCoeffs = (Mat_<float>(5, 1) << -0.60553, 28.6678, 0, 0, -316.316);

	cout << "camera matrix:" << cameraMatrix << endl;
	cout << "distort coefficient:" << distCoeffs << endl;

	// *************************************************************** //
	// Kalman Filter                                                   //
	// *************************************************************** //
	KalmanFilter KF;         // instantiate Kalman Filter
	int nStates = 18;            // the number of states
	int nMeasurements = 6;       // the number of measured states
	int nInputs = 0;             // the number of control actions
	double dt = 0.125;           // time between measurements (1/FPS)

	initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt);    // init function
	Mat measurements(nMeasurements, 1, CV_64F); measurements.setTo(Scalar(0));



	VideoCapture cap(0); // open the default camera
	if (!cap.isOpened()){  // check if we succeeded
		cout << "ERROR: camera cannot be loaded." << endl;
		//return -1;
	}

	vector<vector<Point> > squares;
	vector<vector<Point2f> > squares_2f;
	vector<Point2f> marker;
	vector<Point2f> marker_filter;
	Mat prev_frame;
	int first = 1;


	while (true)
	{
		// retrieve the frame:
		Mat frame, output;
		if (!cap.read(frame)) {
			std::cout << "Unable to retrieve frame from video stream." << std::endl;
			continue;
		}
		// display it:
		//imshow("LiveStream", frame);


		// *************************************************************** //
		// Marker Detection                                                //
		// *************************************************************** //

		// ************ //
		// thresholding //
		// ************ //
		Mat gray_frame, threshframe;
		cvtColor(frame, gray_frame, COLOR_BGR2GRAY);
		Mat diff_frame;
		threshold(gray_frame, threshframe,
			thresh,
			255, THRESH_BINARY_INV);
		medianBlur(threshframe, threshframe, 5);

		Mat threshframe_copy = threshframe;
		//morphological opening (remove small objects from the foreground)
		erode(threshframe_copy, threshframe_copy, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(threshframe_copy, threshframe_copy, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		//morphological closing (fill small holes in the foreground)
		dilate(threshframe_copy, threshframe_copy, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(threshframe_copy, threshframe_copy, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		imshow("threshold_copy", threshframe_copy);
		//mshow("threshold", threshframe);



		// *************************************************************** //
		// Using Sqaure Detection                                          //
		// *************************************************************** //
		vector<vector<Point> > contours;
		//findContours(threshframe, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
		findContours(threshframe_copy, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

		vector<Point2f> approx_2f;
		vector<Point> approx;

		squares.clear();
		squares_2f.clear();
		//marker.clear();

		// test each contour
		for (size_t i = 0; i < contours.size(); i++)
		{
			approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
			approxPolyDP(Mat(contours[i]), approx_2f, arcLength(Mat(contours[i]), true)*0.02, true);
			if (approx.size() == 4 &&
				fabs(contourArea(Mat(approx))) > 10 &&//1000 &&
				isContourConvex(Mat(approx)))
			{
				squares.push_back(approx);
				squares_2f.push_back(approx_2f);
			}
		}


		for (int i = 0; i < squares_2f.size(); i++){
			// ***************************** //
			// extracting region of interest //
			// ***************************** //		

			vector<Point2f> perspective_pt = marekr_image_pt();
			Mat canonicalMarker;
			Mat M = getPerspectiveTransform(squares_2f[i], perspective_pt);
			warpPerspective(gray_frame, canonicalMarker, M, Size(frame.cols, frame.rows));

			threshold(canonicalMarker, canonicalMarker, 125, 255, THRESH_BINARY | THRESH_OTSU);
			Rect myROI(0, 0, 270, 270);
			Mat croppedImage = canonicalMarker(myROI);
			//imshow("cropped", croppedImage); 
			//imshow("perspective", canonicalMarker);

			// *************** //
			// validate marker //
			// *************** //
			Rect subsqr(0, 0, 30, 30);
			Mat subView = croppedImage(subsqr);
			Mat bitMatrix = Mat::zeros(9, 9, CV_8UC1);
			int cellSize = 30;

			for (int y = 0; y < 9; y++)
			{
				for (int x = 0; x<9; x++)
				{
					int cellX = (x)*cellSize;
					int cellY = (y)*cellSize;
					Mat cell = croppedImage(Rect(cellX, cellY, cellSize, cellSize));
					int nZ = cv::countNonZero(cell);
					if (nZ>(cellSize*cellSize) / 2){
						bitMatrix.at<uchar>(y, x) = 1;
					}
				}
			}

			Mat diff;
			Mat rotmarker;

			for (int a = -2; a < 2; a++){
				Mat refmarker;
				Mat markerMatrix = (Mat_<int>(9, 9) <<
					0, 0, 0, 0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0, 0, 0, 0,
					0, 0, 1, 1, 0, 0, 0, 0, 0,
					0, 0, 1, 1, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0, 0, 0, 0);
				markerMatrix.convertTo(refmarker, CV_8UC1);

				//cout << "bitmap\n" << bitMatrix << endl;
				if (a == -2){
					rotmarker = refmarker;
					//cout << "rotation\n" << rotmarker << endl;
				}
				else{
					flip(refmarker, rotmarker, a);
					// -1: both axis
					//  0: x-axis
					//  1: y-axis
					//cout << "rotation\n" << rotmarker << endl;
				}
				compare(bitMatrix, rotmarker, diff, CMP_EQ);

				bool eq = cv::countNonZero(diff) == 81;

				if (eq){

					marker.clear();
					//cornerSubPix(threshframe, squares_2f[i], Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
					if (a == -1){
						//cout << "orientation: reflected to both axis" << endl;
						for (int b = 0; b < 4; b++){
							marker.push_back(squares_2f[i][(b + 2) % 4]);
						}
					}
					else if (a == 0){
						//cout << "orientation: reflected to x-axis" << endl;
						for (int b = 0; b < 4; b++){
							marker.push_back(squares_2f[i][(b + 1) % 4]);
						}
					}
					else if (a == 1){
						//cout << "orientation: reflected to y-axis" << endl;
						for (int b = 0; b < 4; b++){
							marker.push_back(squares_2f[i][(b + 3) % 4]);
						}
					}
					else if (a == -2){
						//cout << "orientation: no reflection" << endl;
						for (int b = 0; b < 4; b++){
							marker.push_back(squares_2f[i][(b) % 4]);
						}
					}
					else{
						cout << "ERROR" << endl;
					}
					squareindex = i;
					drawMarker(frame, squares, squareindex);
					cout << "marker detected" << endl;
					flag = 1;
					KF_flag = true;
					break;

				}
				else{
					KF_flag = false;
				}
			}
		}
		//	drawSquares(frame, squares);

		// *************************************************************** //
		// Pose Estimation                                                 //
		// *************************************************************** //
		vector<Point3f> world_pt = marker_world_pt();	// 3d world coordinates
		vector<Point3f> cube_pt = cube_3d();	//3d cub
		//vector<Point3f> axis = axis_3d();	//3d cub

		Mat R_move, T_move;
		vector<Point2f> cube_2d;
		vector<Point2f> axis_2d;
		vector<Point2f> marker_2d;

		//Mat frame_2 = frame;

		if (marker.size() != 0){
			//cout << marker << endl;

			Mat inliers;
			solvePnPRansac(world_pt, marker, cameraMatrix, distCoeffs, R_move, T_move, false, 500, 2.0, 0.99, inliers, SOLVEPNP_ITERATIVE);
			//cout << "rotation matrix: " << R_move << endl;
			//cout << "translation matrix: " << T_move << endl;
			if (KF_flag){
				Mat translation_measured(3, 1, CV_64F);
				Mat rotation_measured(3, 3, CV_64F);
				translation_measured = T_move;
				Rodrigues(R_move, rotation_measured);
				fillMeasurements(measurements, translation_measured, rotation_measured);
			}


			Mat translation_estimated(3, 1, CV_64F);
			Mat rotation_estimated(3, 3, CV_64F);
			updateKalmanFilter(KF, measurements, translation_estimated, rotation_estimated);


			Mat est_R;
			Mat est_T;
			Mat error_T;
			Rodrigues(rotation_estimated, est_R);
			est_T = translation_estimated;
			//cout << "est rotation matrix: " << est_R << endl;
			//cout << "est translation matrix: " << est_T << endl;

			double dist_t_error = norm(T_move, est_T);
			double dist_r_error = norm(R_move, est_R);

			cout << "difference between the two translation: " << dist_t_error << endl;
			cout << "difference between the two rotation: " << dist_r_error << endl;


			if (dist_t_error > 10 || dist_r_error > 10){
				initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt);    // init function

				Mat measurements(nMeasurements, 1, CV_64F); measurements.setTo(Scalar(0));
			}
			//else {

				projectPoints(cube_pt, est_R, est_T, cameraMatrix, distCoeffs, cube_2d);
				projectPoints(world_pt, est_R, est_T, cameraMatrix, distCoeffs, marker_2d);
				drawCube(frame, cube_2d);

			//}

		}

		if (waitKey(30) == 27)
			break;

	}




	cap.release();
	return 0;
}
