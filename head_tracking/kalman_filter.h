#include "use_opencv.h"

using namespace cv;
using namespace std;

void initKalmanFilter(KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt);
void updateKalmanFilter(KalmanFilter &KF, Mat &measurement, Mat &translation_estimated, Mat &rotation_estimated);
void fillMeasurements(Mat &measurements, const Mat &translation_measured, const Mat &rotation_measured);
cv::Mat euler2rot(const cv::Mat & euler);
cv::Mat rot2euler(const cv::Mat & rotationMatrix);


void initKalmanFilter(KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt)
{

	KF.init(nStates, nMeasurements, nInputs, CV_64F);                 // init Kalman Filter

	setIdentity(KF.processNoiseCov, Scalar::all(1e-3));       // set process noise
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));   // set measurement noise
	setIdentity(KF.errorCovPost, Scalar::all(10));             // error covariance


	/** DYNAMIC MODEL **/

	//  [1 0 0 dt  0  0 dt2   0   0 0 0 0  0  0  0   0   0   0]
	//  [0 1 0  0 dt  0   0 dt2   0 0 0 0  0  0  0   0   0   0]
	//  [0 0 1  0  0 dt   0   0 dt2 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  1  0  0  dt   0   0 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  1  0   0  dt   0 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  0  1   0   0  dt 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  0  0   1   0   0 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  0  0   0   1   0 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  0  0   0   0   1 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  0  0   0   0   0 1 0 0 dt  0  0 dt2   0   0]
	//  [0 0 0  0  0  0   0   0   0 0 1 0  0 dt  0   0 dt2   0]
	//  [0 0 0  0  0  0   0   0   0 0 0 1  0  0 dt   0   0 dt2]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  1  0  0  dt   0   0]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  0  1  0   0  dt   0]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  1   0   0  dt]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   1   0   0]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   1   0]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   0   1]

	// position
	KF.transitionMatrix.at<double>(0, 3) = dt;
	KF.transitionMatrix.at<double>(1, 4) = dt;
	KF.transitionMatrix.at<double>(2, 5) = dt;
	KF.transitionMatrix.at<double>(3, 6) = dt;
	KF.transitionMatrix.at<double>(4, 7) = dt;
	KF.transitionMatrix.at<double>(5, 8) = dt;
	KF.transitionMatrix.at<double>(0, 6) = 0.5*pow(dt, 2);
	KF.transitionMatrix.at<double>(1, 7) = 0.5*pow(dt, 2);
	KF.transitionMatrix.at<double>(2, 8) = 0.5*pow(dt, 2);

	//// orientation
	//KF.transitionMatrix.at<double>(9, 12) = dt;
	//KF.transitionMatrix.at<double>(10, 13) = dt;
	//KF.transitionMatrix.at<double>(11, 14) = dt;
	////KF.transitionMatrix.at<double>(12, 15) = dt;
	//KF.transitionMatrix.at<double>(13, 16) = dt;
	//KF.transitionMatrix.at<double>(14, 17) = dt;
	KF.transitionMatrix.at<double>(9, 15) = 0.5*pow(dt, 2);
	KF.transitionMatrix.at<double>(10, 16) = 0.5*pow(dt, 2);
	KF.transitionMatrix.at<double>(11, 17) = 0.5*pow(dt, 2);


	/** MEASUREMENT MODEL **/

	//  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
	//  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
	//  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
	//  [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
	//  [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
	//  [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]

	KF.measurementMatrix.at<double>(0, 0) = 1;  // x
	KF.measurementMatrix.at<double>(1, 1) = 1;  // y
	KF.measurementMatrix.at<double>(2, 2) = 1;  // z
	KF.measurementMatrix.at<double>(3, 9) = 1;  // roll
	KF.measurementMatrix.at<double>(4, 10) = 1; // pitch
	KF.measurementMatrix.at<double>(5, 11) = 1; // yaw

}

/**********************************************************************************************************/
void updateKalmanFilter(KalmanFilter &KF, Mat &measurement,
	Mat &translation_estimated, Mat &rotation_estimated)
{

	// First predict, to update the internal statePre variable
	Mat prediction = KF.predict();

	// The "correct" phase that is going to use the predicted value and our measurement
	Mat estimated = KF.correct(measurement);

	// Estimated translation
	translation_estimated.at<double>(0) = estimated.at<double>(0);
	translation_estimated.at<double>(1) = estimated.at<double>(1);
	translation_estimated.at<double>(2) = estimated.at<double>(2);

	// Estimated euler angles
	Mat eulers_estimated(3, 1, CV_64F);
	eulers_estimated.at<double>(0) = estimated.at<double>(9);
	eulers_estimated.at<double>(1) = estimated.at<double>(10);
	eulers_estimated.at<double>(2) = estimated.at<double>(11);

	// Convert estimated quaternion to rotation matrix
	rotation_estimated = euler2rot(eulers_estimated);

}

/**********************************************************************************************************/
void fillMeasurements(Mat &measurements,
	const Mat &translation_measured, const Mat &rotation_measured)
{
	// Convert rotation matrix to euler angles
	Mat measured_eulers(3, 1, CV_64F);
	measured_eulers = rot2euler(rotation_measured);

	// Set measurement to predict
	measurements.at<double>(0) = translation_measured.at<double>(0); // x
	measurements.at<double>(1) = translation_measured.at<double>(1); // y
	measurements.at<double>(2) = translation_measured.at<double>(2); // z
	measurements.at<double>(3) = measured_eulers.at<double>(0);      // roll
	measurements.at<double>(4) = measured_eulers.at<double>(1);      // pitch
	measurements.at<double>(5) = measured_eulers.at<double>(2);      // yaw
}

cv::Mat rot2euler(const cv::Mat & rotationMatrix)
{
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
		x = atan2(-m12, m11);
		y = asin(m10);
		z = atan2(-m20, m00);
	}

	euler.at<double>(0) = x;
	euler.at<double>(1) = y;
	euler.at<double>(2) = z;

	return euler;
}


cv::Mat euler2rot(const cv::Mat & euler)
{
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
