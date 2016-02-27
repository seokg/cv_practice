#pragma once
#include "use_opencv.h"

class Blob
{
public:
	std::vector<cv::KeyPoint> blob_detection(cv::Mat frame, cv::Mat threshframe);
	std::vector<cv::Point2f> Blob::ir_contour_detect(cv::Mat frame, cv::Mat threshframe);

	void ir_thresholding(cv::Mat frame, cv::Mat& threshframe, int thresh);

};


