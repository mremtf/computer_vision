#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>

#include "hough_transform.hpp"

int main (int argc, char** argv) {
	if (argc != 2) {
		std::cout << argv[0] << " <input edge image>" << std::endl;
		return -1;
	}

  /// Load an image
	cv::Mat src = cv::imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE);

  if( src.empty() )
    { return -1; }

	if (src.channels() != 1) {
		std::cout << "ERROR failed to convert to 1 channel image" << std::endl;
		return -1;
	}
	cv::Mat img_blurred;
	cv::Mat img_edges;
	
	cv::blur( src, img_blurred, cv::Size(5,5) );
	cv::Canny(img_blurred, img_edges, 100, 150, 3);

	hough_transform ht(img_edges.size());
	ht.transform(img_edges,250);


	return 0;
}
