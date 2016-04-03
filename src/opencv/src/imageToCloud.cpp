#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <unistd.h>

using namespace cv; 
using namespace std; 

int main (int argc, char** argv){

	Mat image = imread("/home/valerian/Documents/src/opencv/test.jpg", 0);
	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.width = image.cols * image.cols;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);
	cout << "Image channels: " << image.channels() << endl;

	for (int row = 0; row < image.rows; row++){
		for (int col = 0; col < image.cols; col++){
			cloud.points[col+row*image.cols].x = col;
			cloud.points[col+row*image.cols].y = row;
			if (image.at<uchar>(row, col) > 0){
				cloud.points[col+row*image.cols].z = 20;
			} else {
				cloud.points[col+row*image.cols].z = 0;
			}
		}
	}

	pcl::io::savePCDFileASCII("pclOut.pcd", cloud);
	cout << "Saved: " << cloud.points.size() << " points to pclOut.pcd" << endl;

	/* Iterators hard to get x,y value
	MatIterator_<uchar> it = image.begin<uchar>();
	MatIterator_<uchar> itend = image.end<uchar>();

	for (;it != itend; it++){
		int a = (int) (*it);
		cout << a << " ";
	}
	*/
	return 0;
}	