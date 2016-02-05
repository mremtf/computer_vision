#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <queue>

#define RAD2DEG(x) (180 *(x) / M_PI)


cv::Mat hysteresis (cv::Mat& gradient, unsigned int threshold_low, unsigned int threshold_high) {
	std::queue<std::pair<int32_t,int32_t> > nodes;
	cv::Mat out = cv::Mat::zeros(gradient.rows,gradient.cols,CV_8UC1);
	std::cout << out.size() << std::endl;
	std::cout << gradient.size() << std::endl;
	for (int32_t r = 0; r < out.rows; ++r) {
		for (int32_t c = 0; c < out.cols; ++c) {
			//std::cout << gradient.at<uchar>(r,c) << std::endl;
			if ((gradient.at<uchar>(r,c) >= threshold_high) && out.at<uchar>(r,c) != 255) {
				nodes.push(std::pair<int32_t,int32_t>(r,c));
				while (!nodes.empty()) {
					std::pair<int32_t,int32_t> node = nodes.front();
					nodes.pop();
					int32_t row = node.first;
					int32_t col = node.second;
					
					if (col < 0 || col >= out.rows || row < 0 || row >= out.cols) {
						continue;
					}
					if (gradient.at<uchar>(row,col) < threshold_low) {
						continue;
					}
					if (out.at<uchar>(row,col) != 255) {
						out.at<uchar>(row,col) = 255;

						nodes.push(std::pair<int32_t,int32_t>(row-1,col-1));
						nodes.push(std::pair<int32_t,int32_t>(row-1,col ));
						nodes.push(std::pair<int32_t,int32_t>(row-1,col+1));
						
						nodes.push(std::pair<int32_t,int32_t>(row ,col+1));
						nodes.push(std::pair<int32_t,int32_t>(row ,col-1));

						nodes.push(std::pair<int32_t,int32_t>(row+1,col-1));
						nodes.push(std::pair<int32_t,int32_t>(row+1,col ));	
						nodes.push(std::pair<int32_t,int32_t>(row+1,col+1));
					}

				}
			}
		}
	}

	return out;
}

int main( int argc, char** argv )
{
	if (argc != 4) {
		std::cout << argv[0] << "<input image> <min threshold> <max threshold>" << std::endl;
		return -1;
	}
  cv::Mat src, src_gray;
  cv::Mat grad;
  const char* window_name = "Result";
  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;
	int t_low = atoi(argv[2]);
	int t_high = atoi(argv[3]);
  /// Load an image
  src = cv::imread( argv[1] );

  if( src.empty() )
    { return -1; }


  cv::GaussianBlur( src, src, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );

  /// Convert it to gray
  cv::cvtColor( src, src_gray, cv::COLOR_RGB2GRAY );

	if (src_gray.channels() != 1) {
		std::cout << "ERROR" << std::endl;
		return -1;
	}

  /// Create window
  cv::namedWindow( window_name, cv::WINDOW_AUTOSIZE );

  /// Generate grad_x and grad_y
 	cv::Mat dx, dy, dx_scaled, dy_scaled;
	
	cv::Mat magnitude (src_gray.rows,src_gray.cols,CV_8UC1);
	cv::Mat orientation (src_gray.rows,src_gray.cols,ddepth);
  /// Gradient X
  //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
  cv::Sobel( src_gray, dx, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );

  /// Gradient Y
  //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
  cv::Sobel( src_gray, dy, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
	
	for (int r = 0 ; r < dy.rows; ++r) {
		for (int c = 0 ; c < dx.cols; ++c)
		{
			const short dxVal = dx.at<short>(r,c);
			const short dyVal = dy.at<short>(r,c);
			magnitude.at<uchar>(r,c) =  sqrt( dxVal * dxVal + dyVal * dyVal);
		}
	}

  // Non-maximum suppression, straightforward implementation.
	cv::Mat nms (src_gray.rows,src_gray.cols,CV_8UC1);

  for (auto r = 1; r < nms.rows - 1; r++) {
  	for (auto c = 1; c < nms.cols - 1; c++) {

			const float dir = (float)(fmod(atan2(dy.at<short>(r,c),
                                                 dx.at<short>(r,c)) + M_PI,
                                           M_PI) / M_PI) * 8;
				// check current vs east and west
				if ((dir <= 1 && dir > 7) && magnitude.at<uchar>(r,c) > magnitude.at<uchar>(r,c + 1) && magnitude.at<uchar>(r,c) > magnitude.at<uchar>(r,c - 1)) {
					nms.at<uchar>(r,c) = magnitude.at<uchar>(r,c);
				}
				// check south east and north west
				else if ((dir > 1 && dir <= 3) && magnitude.at<uchar>(r,c) > magnitude.at<uchar>(r + 1, c + 1) && magnitude.at<uchar>(r,c) > magnitude.at<uchar>(r - 1 ,c - 1)) {
					nms.at<uchar>(r,c) = magnitude.at<uchar>(r,c);
				}
				// check north and south
				else if ((dir > 3 && dir <= 5) && magnitude.at<uchar>(r,c) > magnitude.at<uchar>(r - 1,c) && magnitude.at<uchar>(r,c) > magnitude.at<uchar>(r + 1,c)) {
					nms.at<uchar>(r,c) = magnitude.at<uchar>(r,c);

				}
				// check north east and south west
				else if ((dir > 5 && dir <= 7) && magnitude.at<uchar>(r,c) > magnitude.at<uchar>(r - 1,c + 1) && magnitude.at<uchar>(r,c) > magnitude.at<uchar>(r + 1,c - 1)) {
					nms.at<uchar>(r,c) = magnitude.at<uchar>(r,c);
				}
				else {
					nms.at<uchar>(r,c) = 0;	
				}	
		}
	}

	cv::Mat out = hysteresis(nms,t_low, t_high); 

  cv::imshow ( window_name, out);
	//cv::imshow( "orientation img", )

  cv::waitKey(0);

  return 0;
}
