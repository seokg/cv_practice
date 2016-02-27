#include "ir_track.h"
#include "init_library.h"

bool myfunction(int i, int j) { return (i<j); }
bool IsNumber(double x)
{
	// This looks like it should always be true, 
	// but it's false if x is a NaN.
	return (x == x);
}


void stable(cv::Mat& rotvector){
	cv::Mat rotMatrix;
	cv::Rodrigues(rotvector, rotMatrix);
	double _yaw, _pitch, _roll;
	double yaw, pitch, roll;
	cv::Mat yaw_matrix, pitch_matrix, roll_matrix;
	cv::Mat rotation_matrix(3, 3, CV_64F);
	// Assuming the angles are in radians.
	double* _r = rotMatrix.ptr<double>();
	double rotmatrix[12] = { _r[0], _r[1], _r[2],
		_r[3], _r[4], _r[5],
		_r[6], _r[7], _r[8] };

	if (_r[3] > 0.998) { // singularity at north pole
		_yaw = atan2(_r[2], _r[8]);
		_pitch = M_PI / 2;
		_roll = 0;
	}
	else if (_r[3] < -0.998) { // singularity at south pole
		_yaw = atan2(_r[2], _r[8]);
		_pitch = -M_PI / 2;
		_roll = 0;
	}
	else {
		_roll = atan2(-_r[5], _r[4]);
		_pitch = asin(_r[3]);
		_yaw = atan2(-_r[6], _r[0]);
	}
	yaw = 180 * _yaw / M_PI;
	roll = 180 * _roll / M_PI;
	pitch = 180 * _pitch / M_PI;
	printf("_yaw=%lf, _roll=%lf, _pitch=%lf \n", yaw, roll, pitch);

	//correct roll
	roll = abs(roll);


	yaw_matrix = (cv::Mat_<float>(3, 3) << cos(_yaw), -sin(_yaw), 0
		, sin(_yaw), cos(_yaw), 0
		, 0, 0, 1);
	pitch_matrix = (cv::Mat_<float>(3, 3) << cos(_pitch), 0, sin(_pitch)
		, 0, 1, 0
		, -sin(_pitch), 0, cos(_pitch));
	roll_matrix = (cv::Mat_<float>(3, 3) << 1, 0, 0
		, 0, cos(_roll), -sin(_roll)
		, 0, sin(_roll), cos(_roll));
	rotation_matrix = yaw_matrix * pitch_matrix * roll_matrix;
	cv::Rodrigues(rotation_matrix, rotvector);
}

void rotate(cv::Mat& rotMatrix) {
	double _yaw, _pitch, _roll;
	double yaw, pitch, roll;
	cv::Mat yaw_matrix, pitch_matrix, roll_matrix, rotation_matrix;

	// Assuming the angles are in radians.
	double* _r = rotMatrix.ptr<double>();
	double rotmatrix[12] = { _r[0], _r[1], _r[2],
		_r[3], _r[4], _r[5],
		_r[6], _r[7], _r[8] };

	if (_r[3] > 0.998) { // singularity at north pole
		_yaw = atan2(_r[2], _r[8]);
		_pitch = M_PI / 2;
		_roll = 0;
		return;
	}
	else if (_r[3] < -0.998) { // singularity at south pole
		_yaw = atan2(_r[2], _r[8]);
		_pitch = -M_PI / 2;
		_roll = 0;
		return;
	}
	else {
		_roll = atan2(-_r[5], _r[4]);
		_pitch = asin(_r[3]);
		_yaw = atan2(-_r[6], _r[0]);
	}
	yaw = 180 * _yaw / M_PI;
	roll = 180 * _roll / M_PI;
	pitch = 180 * _pitch / M_PI;
	printf("_yaw=%lf, _roll=%lf, _pitch=%lf \n", yaw, roll, pitch);

	//correct roll
}


void Track::ir_track(){
	int number_LED = 4;
	cv::Mat unstable_R;

	/* comment*/
	double _X, _Y, _Z;

	// OPEN VIDEO
	Video _video;
	cv::VideoCapture cap(0);
	_video.openvideo(cap);

	// CAMERA CALIBRATION
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	cameraMatrix = (cv::Mat_<float>(3, 3) << 889.651, 0, 319.5,
	0, 889.651, 239.5,
	0, 0, 1);
	distCoeffs = (cv::Mat_<float>(5, 1) << -0.06471, 0.4719, 0, 0, -2.24498);

	std::cout << "camera matrix:" << cameraMatrix << std::endl
	<< "distort coefficient:" << distCoeffs << std::endl;

	Blob _blob;
	Kalman _kalman;
	cv::KalmanFilter KF;
	int nStates = 12;//12;           // the number of states
	int nMeasurements = 6;      // the number of measured states
	int nInputs = 0;            // the number of control actions
	double dt = 1 / 8;			// time between measurements (1/FPS)
	cv::Mat measurements(nMeasurements, 1, CV_64F);
	_kalman.initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt);
	int IR_case = 1;
	bool IR_detected;
	bool FIRST = true;
	bool number_flag;
	/* comment end*/


	cv::Mat inliers;
	std::vector<cv::Point3f> world_pt = Track::ir_3d_position();

	//// POSE MATRIX 
	/* comment*/
	cv::Mat R, T, R_kalman, T_kalman;

	cv::Mat translation_measured(3, 1, CV_64F);
	cv::Mat rotation_measured(3, 3, CV_64F);
	cv::Mat translation_estimated(3, 1, CV_64F);
	cv::Mat rotation_estimated(3, 3, CV_64F);
	/* comment end*/
	std::vector<cv::Point3f> IR_world_coordinate = Track::ir_3d_position();
	std::vector<cv::Point3f> Axis_world_coordinate = Track::ir_3d_axis();
	std::vector<cv::Point3f> Cube_world_coordinate = Track::CUBE_position();


	while (true){
		cv::Mat frame;
		cv::Mat copyframe, threshframe;
		int thresh = 10;
		if (!(_video.readvideo(cap, frame))){
			std::cout << "ERROR: unable to read the frame, ending tracking" << std::endl;
			break;
		}
		cv::imshow("live stream", frame);
		copyframe = frame;
		/// IMAGE PROCESSING FILTER
		blob_ir.ir_thresholding(frame, threshframe, thresh);

		/// BLOB DETECTION
		std::vector<cv::Point2f> mc;
		mc.clear();
		mc = blob_ir.ir_contour_detect(frame, threshframe);


		///FIRST CASE
		if (FIRST){
			if (mc.size() == number_LED){
				/// MARKER DETECTED
				for (int i = 0; i < mc.size(); i++){
					// CHECKING NUMBER IS REAL
					if (IsNumber(mc[i].x) && IsNumber(mc[i].y)){
						std::cout << "number OK" << std::endl;
						number_flag = true;
					}
					else{
						std::cout << "number NOT OK" << std::endl;
						number_flag = false;
						break;
					}
				}
				if (number_flag){
					std::cout << "FIRST TIME MARKER DETECTED!!!!" << std::endl;
					///KALMAN FILTER
					IR_case = 2;
					IR_detected = true;
					FIRST = false;
				}
			}
			else{
				/// MARKER NOT DETECTED
				std::cout << "FIRST TIME MARKER NOT DETECTED YET" << std::endl;
				IR_detected = false;
				IR_case = 1;
			}

		}



		/// N CASE
		else{
			if (mc.size() == number_LED){
				/// MARKER DETECTED
				for (int i = 0; i < mc.size(); i++){
					// CHECKING NUMBER IS REAL
					if (IsNumber(mc[i].x) && IsNumber(mc[i].y)){
						std::cout << "number OK" << std::endl;
						number_flag = true;
					}
					else{
						std::cout << "number NOT OK" << std::endl;
						number_flag = false;
						break;
					}
				}
				if (number_flag){
					std::cout << "N TIME MARKER DETECTED!!!!" << std::endl;
					///KALMAN FILTER
					IR_detected = true;
					IR_case = 4;
				}
				else{
					std::cout << "N TIME MARKER NOT DETECTED YET" << std::endl;
					IR_detected = false;
					IR_case = 3;
				}
			}
			else{
				/// MARKER NOT DETECTED
				std::cout << "N TIME MARKER NOT DETECTED YET" << std::endl;
				IR_detected = false;
				IR_case = 3;
			}
		}







		if (IR_detected){
			std::sort(mc.begin(), mc.end(), [](const cv::Point2f &a, const cv::Point2f &b) {
				return (a.y > b.y);
			});


			std::sort(mc.begin(), mc.end(), [](const cv::Point2f &a, const cv::Point2f &b) {
				return (a.x > b.x);
			});
			for (int i = 0; i < mc.size(); i++){
				std::cout << "x:" << mc[i].x <<
					"y:" << mc[i].y << std::endl;
			}
		}



		switch (IR_case){

		case 1: // INIT MARKER NOT DETECTED
			//std::cout << "MARKER NOT DETECTED" << std::endl;
			break;

		case 2: // INIT MARKER  DETECTED
			//std::cout << "MARKER DETECTED" << std::endl;
			// TRACKING
			cv::solvePnPRansac(world_pt, mc, cameraMatrix, distCoeffs, R, T, true, 500, 2.0, 0.99, inliers, cv::SOLVEPNP_ITERATIVE);// cv::SOLVEPNP_ITERATIVE);
			translation_measured = T;
			std::cout << "before: " << R << std::endl;

			//stable(R);
			std::cout << "after: " << R << std::endl;
			cv::Rodrigues(R, rotation_measured);

			std::cout << "T" << translation_measured << std::endl
				<< "R" << rotation_measured << std::endl;

			// KALMAN FILTER
			kalman_ir.fillMeasurements(measurements, translation_measured, rotation_measured);
			kalman_ir.updateKalmanFilter(KF, measurements, translation_estimated, rotation_estimated, IR_detected);

			cv::Rodrigues(rotation_estimated, R_kalman);
			//R_kalman = stable(rotation_estimated);

			T_kalman = translation_estimated;

			// DRAW MARKER 
			cv::projectPoints(IR_world_coordinate, R_kalman, T_kalman, cameraMatrix, distCoeffs, IR_2d);
			cv::projectPoints(Axis_world_coordinate, R_kalman, T_kalman, cameraMatrix, distCoeffs, Axis_2d);
			cv::projectPoints(Cube_world_coordinate, R_kalman, T_kalman, cameraMatrix, distCoeffs, Cube_2d);
			for (int i = 0; i < IR_2d.size(); i++){
				std::cout << "x:" << IR_2d[i].x <<
					"y:" << IR_2d[i].y << std::endl;
			}
			//Track::drawMarker(copyframe, IR_2d);
			Track::drawAxis(copyframe, Axis_2d);
			Track::drawCUBE(frame, Cube_2d);
			break;

		case 3: // IR MARKER NOT DETECTED AND GUESS
			//std::cout << "MARKER NOT DETECTED GUESSING" << std::endl;
			// KALMAN FILTER

			kalman_ir.fillMeasurements(measurements, translation_measured, rotation_measured);
			kalman_ir.updateKalmanFilter(KF, measurements, translation_estimated, rotation_estimated, IR_detected);
			cv::Rodrigues(rotation_estimated, R_kalman);
			T_kalman = translation_estimated;

			// DRAW MARKER 
			cv::projectPoints(IR_world_coordinate, R_kalman, T_kalman, cameraMatrix, distCoeffs, IR_2d);
			cv::projectPoints(Axis_world_coordinate, R_kalman, T_kalman, cameraMatrix, distCoeffs, Axis_2d);
			cv::projectPoints(Cube_world_coordinate, R_kalman, T_kalman, cameraMatrix, distCoeffs, Cube_2d);

			for (int i = 0; i < IR_2d.size(); i++){
				std::cout << "x:" << IR_2d[i].x <<
					"y:" << IR_2d[i].y << std::endl;
			}
			//Track::drawMarker(copyframe, IR_2d);
			Track::drawAxis(copyframe, Axis_2d);
			Track::drawCUBE(frame, Cube_2d);
			break;

		case 4:
			//std::cout << "MARKER DETECTED CORRECTING" << std::endl;
			// TRACKING
			cv::solvePnPRansac(world_pt, mc, cameraMatrix, distCoeffs, R, T, true, 500, 2.0, 0.99, inliers, cv::SOLVEPNP_ITERATIVE);// cv::SOLVEPNP_ITERATIVE);
			translation_measured = T;
			//stable(R);
			cv::Rodrigues(R, rotation_measured);

			// KALMAN FILTER
			kalman_ir.fillMeasurements(measurements, translation_measured, rotation_measured);
			kalman_ir.updateKalmanFilter(KF, measurements, translation_estimated, rotation_estimated, IR_detected);



			cv::Rodrigues(rotation_estimated, R_kalman);
			T_kalman = translation_estimated;

			// DRAW MARKER 
			cv::projectPoints(IR_world_coordinate, R_kalman, T_kalman, cameraMatrix, distCoeffs, IR_2d);
			cv::projectPoints(Axis_world_coordinate, R_kalman, T_kalman, cameraMatrix, distCoeffs, Axis_2d);
			cv::projectPoints(Cube_world_coordinate, R_kalman, T_kalman, cameraMatrix, distCoeffs, Cube_2d);

			for (int i = 0; i < IR_2d.size(); i++){
				std::cout << "x:" << IR_2d[i].x <<
					"y:" << IR_2d[i].y << std::endl;
			}
			//Track::drawMarker(copyframe, IR_2d);
			Track::drawAxis(copyframe, Axis_2d);
			Track::drawCUBE(frame, Cube_2d);
			break;
		default:
			//std::cout << "NONE OF THE CASE WAS DONE" << std::endl;
			break;

		}

		if (IR_case == 2 ||
			IR_case == 3 ||
			IR_case == 4

			){
			double dist_t_error = norm(T_kalman, T);
			double dist_r_error = norm(R_kalman, R);
			//if (dist_t_error > 1000 || dist_r_error > 1000){
			//	kalman_ir.initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt);    // init function
			//	cv::Mat measurements(nMeasurements, 1, CV_64F); measurements.setTo(cv::Scalar(0));
			//}

			cv::Rodrigues(R_kalman, rotation_estimated);
			rotate(rotation_estimated);

			// extract rotation & translation matrix
			cv::Mat R_inv = rotation_estimated.inv();

			cv::Mat P = -R_inv*T_kalman;
			double* position = (double *)P.data;

			// camera position
			printf("x=%lf, y=%lf, z=%lf \n", position[0], position[1], position[2]);

			X = position[0];
			Y = position[1];
			Z = position[2];


			/*cout << "yaw" << yaw << endl
			<< "pitch" << pitch << endl
			<< "roll" << roll << endl;*/
		}
		imshow("Final Output", frame);
		char code = (char)cv::waitKey(10);

		if (code > 0)
			break;
	}
}


std::vector<cv::Point3f> Track::ir_3d_position(){
	/////////////////////////////////
	//          o        o         //
	//                             //
	//    o                    o   //  
	/////////////////////////////////
	std::vector<cv::Point3f> points;	
	float x, y, z;

	x = 15.0; y = 0.0; z = 0.0;		points.push_back(cv::Point3f(x, y, z));
	x = 11.0; y = 8.0; z = 0.0;		points.push_back(cv::Point3f(x, y, z));
	x =  4.0; y = 8.0; z = 0.0;		points.push_back(cv::Point3f(x, y, z));
	x = 0.0; y = 0.0; z = -9.0;		points.push_back(cv::Point3f(x, y, z));

	return points;
}
std::vector<cv::Point3f> Track::ir_3d_axis(){
	std::vector<cv::Point3f> points;
	float x, y, z;
	x = 0.0;  y = 0.0;  z = 0.0;		points.push_back(cv::Point3f(x, y, z));
	x = 0.0;  y = 0.0;  z = -5.0;		points.push_back(cv::Point3f(x, y, z));
	x = 0.0;  y = 5.0; z = 0.0;		points.push_back(cv::Point3f(x, y, z));
	x = 5.0; y = 0.0;  z = 0.0;		points.push_back(cv::Point3f(x, y, z));
	return points;
}
std::vector<cv::Point3f> Track::CUBE_position(){
	/////////////////////////////////
	//          o        o         //
	//                             //
	//    o                    o   //  
	/////////////////////////////////
	std::vector<cv::Point3f> points;
	float x, y, z;
	x = 15.0; y = 0.0; z = 0.0;		points.push_back(cv::Point3f(x, y, z));
	x = 11.0; y = 8.0; z = 0.0;		points.push_back(cv::Point3f(x, y, z));
	x = 4.0; y = 8.0; z = 0.0;		points.push_back(cv::Point3f(x, y, z));
	x = 0.0; y = 0.0; z = 0.0;		points.push_back(cv::Point3f(x, y, z));

	x = 15.0; y = 0.0; z = -5.0;		points.push_back(cv::Point3f(x, y, z));
	x = 11.0; y = 8.0; z = -5.0;		points.push_back(cv::Point3f(x, y, z));
	x = 4.0; y = 8.0; z = -5.0;		points.push_back(cv::Point3f(x, y, z));
	x = 0.0; y = 0.0; z = -5.0;		points.push_back(cv::Point3f(x, y, z));


	return points;
}

void  Track::drawAxis(cv::Mat image, const std::vector<cv::Point2f>axis_2d)
{
	line(image, axis_2d[0], axis_2d[3], cv::Scalar(255, 0, 0), 3); //B : X-axis
	line(image, axis_2d[0], axis_2d[2], cv::Scalar(0, 255, 0), 3); //G : Y-axis
	line(image, axis_2d[0], axis_2d[1], cv::Scalar(0, 0, 255), 3); //R : Z-axis
	imshow("AXIS", image);
}
void  Track::drawMarker(cv::Mat image, const std::vector<cv::Point2f>marker_2d)
{
	line(image, marker_2d[0], marker_2d[1], cv::Scalar(255, 0, 255), 3);
	line(image, marker_2d[1], marker_2d[2], cv::Scalar(255, 0, 255), 3);
	line(image, marker_2d[2], marker_2d[3], cv::Scalar(255, 0, 255), 3);
	line(image, marker_2d[3], marker_2d[0], cv::Scalar(255, 0, 255), 3);
	imshow("CUBE", image);
}
void  Track::drawCUBE(cv::Mat image, const std::vector<cv::Point2f>cube_2d)
{
	//base
	line(image, cube_2d[0], cube_2d[1], cv::Scalar(255, 0, 0), 3);
	line(image, cube_2d[1], cube_2d[2], cv::Scalar(255, 0, 0), 3);
	line(image, cube_2d[2], cube_2d[3], cv::Scalar(255, 0, 0), 3);
	line(image, cube_2d[3], cube_2d[0], cv::Scalar(255, 0, 0), 3);
	//side
	line(image, cube_2d[0], cube_2d[4], cv::Scalar(0, 0, 255), 3);
	line(image, cube_2d[1], cube_2d[5], cv::Scalar(0, 255, 0), 3);
	line(image, cube_2d[2], cube_2d[6], cv::Scalar(255, 0, 0), 3);
	line(image, cube_2d[3], cube_2d[7], cv::Scalar(255, 255, 255), 3);
	//top
	line(image, cube_2d[4], cube_2d[5], cv::Scalar(0, 0, 255), 3);
	line(image, cube_2d[5], cube_2d[6], cv::Scalar(0, 0, 255), 3);
	line(image, cube_2d[6], cube_2d[7], cv::Scalar(0, 0, 255), 3);
	line(image, cube_2d[7], cube_2d[4], cv::Scalar(0, 0, 255), 3);
	imshow("CUBE", image);
}





