#include "kalman_lib.h"
void Kalman::initKalmanFilter(cv::KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt){




	KF.init(nStates, nMeasurements, nInputs, CV_64F);                 // init Kalman Filter

	setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));       // set process noise
	setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-5));   // set measurement noise
	setIdentity(KF.errorCovPost, cv::Scalar::all(0.01));             // error covariance


	/** DYNAMIC MODEL **/
	//     0 1 2  3  4  5   6   7   8 9  0  1  
	//
	//0   [1 0 0 dt  0  0   0   0   0 0  0  0 ]
	//1   [0 1 0  0 dt  0   0   0   0 0  0  0 ]
	//2   [0 0 1  0  0 dt   0   0   0 0  0  0 ]
	//3   [0 0 0  1  0  0   0   0   0 0  0  0 ]
	//4   [0 0 0  0  1  0   0   0   0 0  0  0 ]
	//5   [0 0 0  0  0  1   0   0   0 0  0  0 ]
	//6   [0 0 0  0  0  0   1   0   0 dt 0  0 ]
	//7   [0 0 0  0  0  0   0   1   0 0  dt 0 ]
	//8   [0 0 0  0  0  0   0   0   1 0  0  dt]
	//9   [0 0 0  0  0  0   0   0   0 1  0  0 ]
	//10  [0 0 0  0  0  0   0   0   0 0  1  0 ]
	//11  [0 0 0  0  0  0   0   0   0 0  0  1 ]


	// position
	KF.transitionMatrix.at<double>(0, 3) = dt;
	KF.transitionMatrix.at<double>(1, 4) = dt;
	KF.transitionMatrix.at<double>(2, 5) = dt;

	//// orientation
	KF.transitionMatrix.at<double>(6, 9) = dt;
	KF.transitionMatrix.at<double>(7, 10) = dt;
	KF.transitionMatrix.at<double>(8, 11) = dt;


	/** MEASUREMENT MODEL **/

	//  [1 0 0 0 0 0 0 0 0 0 0 0 ]
	//  [0 1 0 0 0 0 0 0 0 0 0 0 ]
	//  [0 0 1 0 0 0 0 0 0 0 0 0 ]
	//  [0 0 0 0 0 0 1 0 0 0 0 0 ]
	//  [0 0 0 0 0 0 0 1 0 0 0 0 ]
	//  [0 0 0 0 0 0 0 0 1 0 0 0 ]

	KF.measurementMatrix.at<double>(0, 0) = 1;  // x
	KF.measurementMatrix.at<double>(1, 1) = 1;  // y
	KF.measurementMatrix.at<double>(2, 2) = 1;  // z
	KF.measurementMatrix.at<double>(3, 6) = 1;  // roll
	KF.measurementMatrix.at<double>(4, 7) = 1; // pitch
	KF.measurementMatrix.at<double>(5, 8) = 1; // yaw
}
void Kalman::updateKalmanFilter(cv::KalmanFilter &KF, cv::Mat &measurement, cv::Mat &translation_estimated, cv::Mat &rotation_estimated, bool ir_detected){
	// First predict, to update the internal statePre variable
	cv::Mat prediction = KF.predict();

	//KF.statePre.at<float>(0);
	//KF.statePre.at<float>(1);
	//KF.statePre.at<float>(2);
	//KF.statePre.at<float>(3);
	//KF.statePre.at<float>(4);
	//KF.statePre.at<float>(5);

	// The "correct" phase that is going to use the predicted value and our measurement
	if (ir_detected){
		cv::Mat estimated = KF.correct(measurement);


		//KF.temp5.at<float>(0) = measurement.at<float>(0) - KF.statePre.at<float>(0);

		// Estimated translation
		translation_estimated.at<double>(0) = estimated.at<double>(0);
		translation_estimated.at<double>(1) = estimated.at<double>(1);
		translation_estimated.at<double>(2) = estimated.at<double>(2);

		// Estimated euler angles
		cv::Mat eulers_estimated(3, 1, CV_64F);
		eulers_estimated.at<double>(0) = estimated.at<double>(6);
		eulers_estimated.at<double>(1) = estimated.at<double>(7);
		eulers_estimated.at<double>(2) = estimated.at<double>(8);

		// Convert estimated quaternion to rotation matrix
		rotation_estimated = euler2rot(eulers_estimated);
	}
}
void Kalman::fillMeasurements(cv::Mat &measurements, const cv::Mat &translation_measured, const cv::Mat &rotation_measured){
	// Convert rotation matrix to euler angles
	cv::Mat measured_eulers(3, 1, CV_64F);
	measured_eulers = rot2euler(rotation_measured);

	// Set measurement to predict
	measurements.at<double>(0) = translation_measured.at<double>(0); // x
	measurements.at<double>(1) = translation_measured.at<double>(1); // y
	measurements.at<double>(2) = translation_measured.at<double>(2); // z
	measurements.at<double>(3) = measured_eulers.at<double>(0);      // roll
	measurements.at<double>(4) = measured_eulers.at<double>(1);      // pitch
	measurements.at<double>(5) = measured_eulers.at<double>(2);      // yaw
}
//roll, pitch, jaw




cv::Mat Kalman::rot2euler(const cv::Mat & rotationMatrix){
	cv::Mat euler(3, 1, CV_64F);

	double m00 = rotationMatrix.at<double>(0, 0);
	double m02 = rotationMatrix.at<double>(0, 2);
	double m10 = rotationMatrix.at<double>(1, 0);
	double m11 = rotationMatrix.at<double>(1, 1);
	double m12 = rotationMatrix.at<double>(1, 2);
	double m20 = rotationMatrix.at<double>(2, 0);
	double m22 = rotationMatrix.at<double>(2, 2);

	double x, y, z;

	// Assuming the angles are in radians.
	if (m10 > 0.998) { // singularity at north pole
		x = 0;
		y = CV_PI / 2;
		z = atan2(m02, m22);
	}
	else if (m10 < -0.998) { // singularity at south pole
		x = 0;
		y = -CV_PI / 2;
		z = atan2(m02, m22);
	}
	else
	{
		//roll, pitch, jaw
		x = atan2(-m12, m11);
		y = asin(m10);
		z = atan2(-m20, m00);
	}

	euler.at<double>(0) = x;
	euler.at<double>(1) = y;
	euler.at<double>(2) = z;

	return euler;
}

cv::Mat Kalman::euler2rot(const cv::Mat & euler){
	cv::Mat rotationMatrix(3, 3, CV_64F);

	double x = euler.at<double>(0);
	double y = euler.at<double>(1);
	double z = euler.at<double>(2);

	// Assuming the angles are in radians.
	double ch = cos(z);
	double sh = sin(z);
	double ca = cos(y);
	double sa = sin(y);
	double cb = cos(x);
	double sb = sin(x);

	double m00, m01, m02, m10, m11, m12, m20, m21, m22;

	m00 = ch * ca;
	m01 = sh*sb - ch*sa*cb;
	m02 = ch*sa*sb + sh*cb;
	m10 = sa;
	m11 = ca*cb;
	m12 = -ca*sb;
	m20 = -sh*ca;
	m21 = sh*sa*cb + ch*sb;
	m22 = -sh*sa*sb + ch*cb;

	rotationMatrix.at<double>(0, 0) = m00;
	rotationMatrix.at<double>(0, 1) = m01;
	rotationMatrix.at<double>(0, 2) = m02;
	rotationMatrix.at<double>(1, 0) = m10;
	rotationMatrix.at<double>(1, 1) = m11;
	rotationMatrix.at<double>(1, 2) = m12;
	rotationMatrix.at<double>(2, 0) = m20;
	rotationMatrix.at<double>(2, 1) = m21;
	rotationMatrix.at<double>(2, 2) = m22;

	return rotationMatrix;
}


