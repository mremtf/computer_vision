#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <cstdlib>
#include "optical_flow.hpp"


const char* ORIG_IMG = "Input";
const char* LINES = "Velocity Image";
const char* ACCUM = "Output Image";

int main (int argc, char** argv) {
	if (argc != 3) {
		std::cout << argv[0] << " <input image>" << std::endl;
		return -1;
	}

/*	int threshold = atoi(argv[2]);
	int radius = atoi(argv[3]);

  /// Load an image
	cv::Mat src = cv::imread( argv[1], 1);

	cv::Mat img_blurred;
	cv::Mat img_edges;	
	cv::Mat img_res = src.clone();
	cv::blur( src, img_blurred, cv::Size(5,5) );
	cv::Canny(img_blurred, img_edges, 100, 150, 3);

	cv::namedWindow(LINES, 	 cv::WINDOW_AUTOSIZE);
	cv::namedWindow(EDGE,	 cv::WINDOW_AUTOSIZE);
	cv::namedWindow(ORIG_IMG, cv::WINDOW_AUTOSIZE);
	cv::namedWindow(ACCUM,	 cv::WINDOW_AUTOSIZE);
	
	cv::imshow(ORIG_IMG, src);	
	cv::imshow(EDGE, img_edges);
	cv::imshow(LINES, img_res);	
	cv::imshow(ACCUM, img_accum);

	cv::waitKey(0);

	std::stringstream ss;
	ss << ORIG_IMG << ".jpg";
  cv::imwrite ( ss.str().c_str(), src);
	ss.str("");

	ss << EDGE << ".jpg";
  cv::imwrite ( ss.str().c_str(), img_edges);
	ss.str("");
	
	ss << LINES << ".jpg";
  cv::imwrite ( ss.str().c_str(), img_res);
  ss.str("");

	ss << ACCUM << ".jpg";
  cv::imwrite ( ss.str().c_str(), img_accum);
  ss.str("");*/
	
	return 0;
}
