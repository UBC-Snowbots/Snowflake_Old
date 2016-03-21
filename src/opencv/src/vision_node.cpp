#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include "filter.h"
#include "IPM.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

class vision_node
{
	snowbotsFilter filter = snowbotsFilter(98, 130, 30, 165, 129, 255);
	std::vector<cv::Point2f> origPoints;
	std::vector<cv::Point2f> dstPoints;
	IPM ipm;

	public:
	vision_node(int width, int height);


};

vision_node::vision_node(int width, int height){
	origPoints.push_back( cv::Point2f(0, height));
    origPoints.push_back( cv::Point2f(width, height));
    origPoints.push_back( cv::Point2f(width/2+100, 200));
    origPoints.push_back( cv::Point2f(width/2-100, 200));
    dstPoints.push_back( cv::Point2f(0, height) );
    dstPoints.push_back( cv::Point2f(width, height) );
    dstPoints.push_back( cv::Point2f(width, 0) );
    dstPoints.push_back( cv::Point2f(0, 0) );
    ipm = IPM(cv::Size(width, height), cv::Size(width,height), origPoints, dstPoints);
}

int main(void){
	
}