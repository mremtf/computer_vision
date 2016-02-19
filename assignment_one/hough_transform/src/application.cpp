#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <cstdlib>
#include "hough_transform.hpp"


const char* ORIG_IMG = "Input";
const char* LINES = "Result";
const char* EDGE = "Canny Edge Detection";
const char* ACCUM = "Polar Space";

int main (int argc, char** argv) {
	if (argc != 4) {
		std::cout << argv[0] << " <input edge image> <threshold> <window radius>" << std::endl;
		return -1;
	}

	int threshold = atoi(argv[2]);
	int radius = atoi(argv[3]);

  /// Load an image
	cv::Mat src = cv::imread( argv[1], 1);

	//cv::Mat src_gray = src;
	/// Convert it to gray to 1 channel image
  //cv::cvtColor( src, src_gray, cv::COLOR_RGB2GRAY );

  /*if( src_gray.empty() )
  { 
		std::cout << "FAILED TO OPEN FILE" << std::endl;
		return -1; 
		
	}*/

	/*if (src_gray.channels() != 1) {
		std::cout << "ERROR failed to convert to 1 channel image" << std::endl;
		return -1;
	}*/
	cv::Mat img_blurred;
	cv::Mat img_edges;	
	cv::Mat img_res = src.clone();
	cv::blur( src, img_blurred, cv::Size(5,5) );
	cv::Canny(img_blurred, img_edges, 100, 150, 3);

	if (img_edges.channels() != 1) {
		std::cout << "not a one band image :(" << std::endl;
		return -1;
	}

	hough_transform ht(img_edges.size());
	if (ht.transform(img_edges,250) != 0) {
		std::cout << "Transform Failed" << std::endl;
		return -1;
	}
	// get the lines 
	std::vector<std::pair<cv::Point,cv::Point> > lines = ht.find_lines(threshold,radius);
	std::cout << "LINES SIZE " << lines.size() << std::endl;
	std::vector<std::pair<cv::Point,cv::Point> >::iterator it;
	it = lines.begin();
	for (; it != lines.end(); it++) {
		cv::line(img_res,it->first,it->second, cv::Scalar(0,0,255), 2,8);
	}

	Accumulator accum = ht.accumulator();

	const unsigned int size = accum.height * accum.width;
	int max = 0;
	for(int v=0; v < size;v++)
		{
			if((int)accum.accumulator[v] > max)
				max = accum.accumulator[v];
		}
		double contrast = 5.0;
		double coef = 255.0 / (double)max * contrast;


	cv::Mat img_accum(accum.height,accum.width,CV_8UC3);
	for (auto i = 0; i < size; ++i) {
		unsigned char c = (double)accum.accumulator[i] * coef < 255.0 ? (double)accum.accumulator[i] * coef : 255.0;	
		img_accum.data[(i*3)+2] = 255;
		img_accum.data[(i*3)+1] = 255-c;
		img_accum.data[(i*3)+0] = 255-c;
	}

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
  ss.str("");
	
	return 0;
}
