#include <opencv2/highgui.hpp>
#include "ros/ros.h"
#include <opencv2/core/core.hpp>

using namespace cv;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imageload");
  ros::NodeHandle n;
  Mat img = imread("/home/vagrant/src/opencvshenanigans/sky.jpg",CV_LOAD_IMAGE_COLOR);
  imshow("opencvtest",img);
  waitKey(0);
  return 0;
}