#pragma once
#include "use_opencv.h"
class Video
{
public:
	static bool openvideo(cv::VideoCapture cap);
	static bool readvideo(cv::VideoCapture cap, cv::Mat& frame);
	static void closevideo(cv::VideoCapture cap);
};