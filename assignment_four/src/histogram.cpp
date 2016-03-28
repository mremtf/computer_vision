#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

/**
 * @function main
 */
int main( int argc, char** argv )
{
	if (argc != 3) {
		std::cout << argv[0] << " <input_image> <threshold_value> " << std::endl;
		return -1;
	}
  Mat src, dst;

  /// Load image
  src = imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE );
	int threshold = atoi(argv[2]);
  if( !src.data )
    { return -1; }

  /// Establish the number of bins
  int histSize = 256;

  /// Set the ranges ( for B,G,R) )
  //float range[] = { threshold, 256 };
	float* range = new float[2];
	range[0] = threshold;
	range[1] = 255;
	float** histRange = &range;
  bool uniform = true; bool accumulate = false;

  Mat hist;

  /// Compute the histograms:
  calcHist( &src, 1, 0, Mat(), hist, 1, &histSize, (const float**) histRange, uniform, accumulate );
	delete[] range;
	histRange = 0;

  // Draw the histograms for B, G and R
  int hist_w = 512; int hist_h = 400;
  int bin_w = cvRound( (double) hist_w/histSize );

  Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

  /// Normalize the result to [ 0, histImage.rows ]
  normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

  /// Draw for each channel
  for( int i = 1; i < histSize; i++ )
  {
      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1)) ) ,
                       Point( bin_w*(i), hist_h - cvRound(hist.at<float>(i)) ),
                       Scalar( 255, 0, 0), 2, 8, 0  );
  }

  /// Display
  namedWindow("Histogram", CV_WINDOW_AUTOSIZE );
  imshow("Histogram", histImage );
  waitKey(0);

  imwrite("Histogram.jpg", histImage );
  return 0;
}
