#pragma once
#include "use_opencv.h"
class Track 
{
public:
	int number_LED = 4;
	cv::Mat unstable_R;
	double _X, _Y, _Z;

	static void ir_track();
	static std::vector<cv::Point3f> ir_3d_position();
	static std::vector<cv::Point3f> ir_3d_axis();
	static std::vector<cv::Point3f> CUBE_position();
	static void drawAxis(cv::Mat image, const std::vector<cv::Point2f>axis_2d);
	static void drawMarker(cv::Mat image, const std::vector<cv::Point2f>marker_2d);
	static void drawCUBE(cv::Mat image, const std::vector<cv::Point2f>cube_2d);
};