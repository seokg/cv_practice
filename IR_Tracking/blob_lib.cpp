#include "blob_lib.h"
std::vector<cv::KeyPoint> Blob::blob_detection(cv::Mat frame, cv::Mat threshframe){

	std::vector<cv::KeyPoint> keyPoints;
	std::vector<std::vector<cv::Point>> approxContours;
	std::vector<std::vector<cv::Point>> contours;

	cv::Mat canny, result;


	//** BLOB PARAMETER **//
	cv::SimpleBlobDetector::Params detectorParams;
	detectorParams.thresholdStep = 20;
	detectorParams.minThreshold = 10;
	detectorParams.maxThreshold = 50;
	detectorParams.minDistBetweenBlobs = 2;

	detectorParams.filterByArea = true;
	detectorParams.minArea = 50;
	detectorParams.maxArea = 300;

	detectorParams.filterByConvexity = true;
	detectorParams.minConvexity = 0.3;
	detectorParams.maxConvexity = 10;

	detectorParams.filterByInertia = true;
	detectorParams.minInertiaRatio = 0.01;
	detectorParams.maxInertiaRatio = 10;

	detectorParams.filterByColor = true;
	detectorParams.blobColor = 255;
	// blobColor = 0 to extract dark blobs 
	// blobColor = 255 to extract light blobs.

	detectorParams.filterByCircularity = true;
	detectorParams.minCircularity = 0.01;
	detectorParams.maxCircularity = 10;
	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(detectorParams);
	detector->detect(threshframe, keyPoints);
	result = cv::Mat(cv::Size(frame.cols, frame.rows), frame.type());
	cv::drawKeypoints(frame, keyPoints, result, CV_RGB(255, 0, 0), cv::DrawMatchesFlags::DEFAULT);

	cv::imshow("key points", result);
	for (int i = 0; i < keyPoints.size(); i++){
		float X = keyPoints[i].pt.x;
		float Y = keyPoints[i].pt.y;
		std::cout << i << ":"  << X <<","<< Y << std::endl;
	}
	return keyPoints;

}


std::vector<cv::Point2f> Blob::ir_contour_detect(cv::Mat frame, cv::Mat threshframe){
	cv::RNG rng(12345);
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	///find contour
	cv::findContours(threshframe, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	

	/// Get the moments
	
	std::vector<cv::Moments> mu(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		mu[i] = moments(contours[i], false);
	}

	///  Get the mass centers:
	std::vector<cv::Point2f> mc(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		mc[i] = cv::Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
	}
	
	/// Draw contours
	cv::Mat drawing = cv::Mat::zeros(threshframe.size(), CV_8UC3);
	for (int i = 0; i< contours.size(); i++)
	{
		cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		cv::drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
		cv::circle(drawing, mc[i], 4, color, -1, 8, 0);
	}


	/// Show in a window
	cv::namedWindow("Contours", CV_WINDOW_AUTOSIZE);
	cv::imshow("Contours", drawing);


	//if (mc.size() == 4){


	//	std::cout << "UNSORTED COORDINATE" << std::endl;

	//	for (int i = 0; i < mc.size(); i++){

	//		std::cout << "x:" << mc[i].x <<
	//			"y:" << mc[i].y << std::endl;
	//	}


	//}
	return mc;

}

void Blob::ir_thresholding(cv::Mat frame, cv::Mat& threshframe, int thresh){
	cv::Mat gray_frame;
	cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
	//cv::equalizeHist(gray_frame, gray_frame);

	dilate(gray_frame, gray_frame, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 11)));
	imshow("gray", gray_frame);
	cv::threshold(gray_frame, threshframe, thresh, 255, cv::THRESH_BINARY);
	cv::imshow("threshold", threshframe);
}
