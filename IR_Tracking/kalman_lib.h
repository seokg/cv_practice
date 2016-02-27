#pragma once
#include "use_opencv.h"

class Kalman
{
public:
	static void initKalmanFilter(cv::KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt);
	static void updateKalmanFilter(cv::KalmanFilter &KF, cv::Mat &measurement, cv::Mat &translation_estimated, cv::Mat &rotation_estimatedbool, bool ir_detected);
	static void fillMeasurements(cv::Mat &measurements, const cv::Mat &translation_measured, const cv::Mat &rotation_measured);
	static cv::Mat euler2rot(const cv::Mat & euler);
	static cv::Mat rot2euler(const cv::Mat & rotationMatrix);
};
