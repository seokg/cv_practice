#pragma once
#include "use_opencv.h"
#include "kalman_lib.h"
#include "ir_track.h"
#include "blob_lib.h"
#include "video_lib.h"

#define M_PI    3.14159265358979323846

// VIDEO
cv::VideoCapture cap(0);
cv::Mat frame;// = new cv::Mat();


// CALIBRATION
cv::Mat cameraMatrix, distCoeffs;

std::vector<std::vector<cv::Point2f> > imagePoints;
std::vector<cv::Mat> rvecs; 
std::vector<cv::Mat> tvecs;
std::vector<float> reprojErrs;
double totalAvgErr;

// POSITION
double X, Y, Z, roll, pitch, yaw;


//Blob_Detection::Blob _blob;
// MARKER 
Track track_ir;
Blob blob_ir;
int IR_case = 1;
bool IR_detected;
bool FIRST = true;
bool number_flag;
std::vector<cv::Point3f> IR_world_coordinate;
std::vector<cv::Point3f> Axis_world_coordinate;
std::vector<cv::Point3f> Cube_world_coordinate;

std::vector<cv::Point2f> Axis_2d;
std::vector<cv::Point2f> IR_2d;
std::vector<cv::Point2f> Cube_2d;

// KALMAN
Kalman kalman_ir;
cv::KalmanFilter KF;
int nStates = 12;           // the number of states
int nMeasurements = 6;      // the number of measured states
int nInputs = 0;            // the number of control actions
double dt = 1 / 8;			// time between measurements (1/FPS)
cv::Mat measurements(nMeasurements, 1, CV_64F);


// POSE MATRIX 
cv::Mat R, T, R_kalman, T_kalman;

cv::Mat translation_measured(3, 1, CV_64F);
cv::Mat rotation_measured(3, 3, CV_64F);
cv::Mat translation_estimated(3, 1, CV_64F);
cv::Mat rotation_estimated(3, 3, CV_64F);