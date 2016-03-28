/*
* Matthew England
**/

#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;

int main (int argc, char** argv) {
	if (argc != 3) {
		std::cout << argv[0] << " <input_image> <threshold_value> " << std::endl;
		return -1;
	}

	int threshold = atoi(argv[2]);
	Mat img = imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE );

	if (!img.data) {
		std::cout << "no input image" << std::endl;
		return -1;

	}
	
	Mat copy = img.clone();
	
	for (auto r = 0; r < img.rows; ++r) {
		for (auto c = 0; c < img.cols; ++c) {
			if (img.at<uchar>(r,c) <= threshold) {
				copy.at<uchar>(r,c) = 0;
			}
			else{
				copy.at<uchar>(r,c) = 255;
			}
		}
	}
	
	imwrite("threshold.jpg", copy);
	return 0;
}
