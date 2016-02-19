/*
 * Takes in an image and transforms it into a binary image
 * given some color specification.
 * Author: Valerian Ratu
 * Ref: 	Color Picker
 *			https://raw.githubusercontent.com/kylehounslow/opencv-tuts/master/auto-colour-filter/AutoColourFilter.cpp
 *			Color Bar:
 *			http://opencv-srf.blogspot.ca/2010/09/object-detection-using-color-seperation.html
 */

#include "filter.h"

//Public
 
//Two different constructors
snowbotsFilter::snowbotsFilter(){
	createFilter(0, 179, 0, 255, 0, 255);
}

snowbotsFilter::snowbotsFilter(int iLowH, int iHighH, int iLowS, int iHighS, int iLowV, int iHighV){
	createFilter(iLowH, iHighH, iLowS, iHighS, iLowV, iHighV);
}


void snowbotsFilter::calibrateWindow(const cv::Mat &input){
	if (calibrationMode && !calibrationImgSet){
		calibrationImage = input.clone();
		cv::cvtColor(calibrationImage, hsv_calibrationImage, CV_BGR2HSV);
		cv::namedWindow(calibrationWindow, CV_WINDOW_AUTOSIZE);
		//setMouseCallback(calibrationWindow, clickAndDrag_Rectangle, &hsv_calibrationImage);
		calibrationImgSet = true;
	}
	else if (calibrationMode){
		cv::imshow(calibrationWindow, calibrationImage);
	}
	else {
		cv::destroyWindow(calibrationWindow);
	}
}

//Private
//Initializer
void snowbotsFilter::createFilter(int iLowH, int iHighH, int iLowS, int iHighS, int iLowV, int iHighV){
	_iLowH = iLowH;
	_iHighH = iHighH;
	_iLowS = iLowS;
	_iHighS = iHighS;
	_iLowV = iLowV;
	_iHighV = iHighV;
	manualCalibrationWindow = "Manual Calibration";
	calibrationWindow = "Frame Calibration";

}


void snowbotsFilter::manualCalibration(){
	cv::namedWindow(manualCalibrationWindow, CV_WINDOW_AUTOSIZE);
    cv::createTrackbar("LowH", manualCalibrationWindow, &_iLowH, 179); //Hue (0 - 179)
    cv::createTrackbar("HighH", manualCalibrationWindow, &_iHighH, 179);

    cv::createTrackbar("LowS", manualCalibrationWindow, &_iLowS, 255); //Saturation (0 - 255)
    cv::createTrackbar("HighS", manualCalibrationWindow, &_iHighS, 255);

    cv::createTrackbar("LowV", manualCalibrationWindow, &_iLowV, 255); //Value (0 - 255)
    cv::createTrackbar("HighV", manualCalibrationWindow, &_iHighV, 255);
}

void snowbotsFilter::stopManualCalibration(){
	cv::destroyWindow(manualCalibrationWindow);
}

void snowbotsFilter::filterImage(const cv::Mat &input, cv::Mat &output){
	cv::cvtColor(input, output, CV_BGR2HSV);
	cv::inRange(output, cv::Scalar(_iLowH, _iLowS, _iLowV), cv::Scalar(_iHighH, _iHighS, _iHighV), output);

	//Morphological Opening (removes small objects from foreground)
    cv::erode(output, output, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
    cv::dilate(output, output, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));

    //Morphological Closing (fill small holes in the foreground)
    cv::dilate(output, output, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
    cv::erode(output, output, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
}

void snowbotsFilter::printValues(void){
	std::cout << "iLowH: " << _iLowH << std::endl;
	std::cout << "iHighH: " << _iHighH << std::endl;
	std::cout << "iLowS: " << _iLowS << std::endl;
	std::cout << "iHighS: " << _iHighS << std::endl;
	std::cout << "iLowV: " << _iLowV << std::endl;
	std::cout << "iHighV: " << _iHighV << std::endl;

}

