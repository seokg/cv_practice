#include "video_lib.h"


bool Video::openvideo(cv::VideoCapture cap){
	if (!(cap).isOpened()){
		std::cout << "ERROR: camera cannot be loaded." << std::endl;
		return false;
	}
	else{
		std::cout << "Camera Opened." << std::endl;
		return true;
	}
}

bool Video::readvideo(cv::VideoCapture cap,cv::Mat& frame){
	if (!cap.read(frame)) {
		std::cout << "Unable to retrieve frame from video stream." << std::endl;
		return false;
	}
	else
		return true;
}

void Video::closevideo(cv::VideoCapture cap){
	cap.release();
}