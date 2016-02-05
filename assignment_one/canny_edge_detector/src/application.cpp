/*
* Matthew England
*	2/5/2016
* Canny Edge Detector
**/

#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <queue>
#include <sstream>

// Does connected componments as strong edges that are above the threshold low with edges that are 
// above the threshold high. The edge intensities above the threshold high are called strong edges
cv::Mat hysteresis (cv::Mat& gradient, unsigned int threshold_low, unsigned int threshold_high) {
	std::queue<std::pair<int32_t,int32_t> > nodes;
	cv::Mat out = cv::Mat::zeros(gradient.rows,gradient.cols,CV_8UC1);
	for (int32_t r = 0; r < out.rows; ++r) {
		for (int32_t c = 0; c < out.cols; ++c) {
			// Is the current pixel a strong edge and not visisted yet
			if ((gradient.at<uchar>(r,c) >= threshold_high) && out.at<uchar>(r,c) != 255) {
				// push onto queue
				nodes.push(std::pair<int32_t,int32_t>(r,c));
				// add all edges not visited
				while (!nodes.empty()) {
					std::pair<int32_t,int32_t> node = nodes.front();
					nodes.pop();
					int32_t row = node.first;
					int32_t col = node.second;
					// avoid going out of bounds of the image
					if (col < 0 || col >= out.rows || row < 0 || row >= out.cols) {
						continue;
					}
					// are you a too weak edge to be considered
					if (gradient.at<uchar>(row,col) < threshold_low) {
						continue;
					}
					// Not visisted yet edge
					if (out.at<uchar>(row,col) != 255) {
						// now visisted
						out.at<uchar>(row,col) = 255;
						// add are 8 directions from the pixel
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

// Canny Edge Detector
int main( int argc, char** argv )
{
	if (argc != 5) {
		std::cout << argv[0] << " <input image> <min threshold> <max threshold> <sigma>" << std::endl;
		return -1;
	}
  cv::Mat src, src_gray;
  cv::Mat grad;
	const char* window_name_one = "Input";
	const char* window_name_two = "Gradient";
	const char* window_name_three = "Non Maximum Suppression";
  const char* window_name_four = "Final";
  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;
	int t_low = atoi(argv[2]);
	if (t_low < 1) {
		std::cout << "threshold low needs to be 1 or higher" << std::endl;
		return -1;
	}
	int t_high = atoi(argv[3]);
	if (t_high < 1 || t_high >= 256) {
		std::cout << "threshold high needs to be 1 or higher, but less than 255" << std::endl;
		return -1;	
	}

	float sigma = atof(argv[4]);
  /// Load an image
  src = cv::imread( argv[1] );

  if( src.empty() )
    { return -1; }

	// Apply the gaussian filter on the input image
  cv::GaussianBlur( src, src, cv::Size(3,3), sigma, sigma, cv::BORDER_DEFAULT );

  /// Convert it to gray to 1 channel image
  cv::cvtColor( src, src_gray, cv::COLOR_RGB2GRAY );

	if (src_gray.channels() != 1) {
		std::cout << "ERROR failed to convert to 1 channel image" << std::endl;
		return -1;
	}

  /// Create windows
  cv::namedWindow( window_name_one, cv::WINDOW_AUTOSIZE );	
  cv::namedWindow( window_name_two, cv::WINDOW_AUTOSIZE );
  cv::namedWindow( window_name_three, cv::WINDOW_AUTOSIZE );
  cv::namedWindow( window_name_four, cv::WINDOW_AUTOSIZE );
  /// Generate grad_x and grad_y
 	cv::Mat dx, dy, dx_scaled, dy_scaled;
	
	cv::Mat magnitude (src_gray.rows,src_gray.cols,CV_8UC1);
	cv::Mat orientation (src_gray.rows,src_gray.cols,ddepth);
  /// calculate Gradient X
  cv::Sobel( src_gray, dx, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );

  /// calculate Gradient Y
  cv::Sobel( src_gray, dy, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
	// Calculate the magnitude of the edges	
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
	

	// Find the local maximums of the edges produced from the gradients
  for (auto r = 1; r < nms.rows - 1; r++) {
  	for (auto c = 1; c < nms.cols - 1; c++) {

			float dir = atan2(dy.at<short>(r,c),dx.at<short>(r,c));
			if (dir < 0) {
				dir += M_PI;
			}
    	// check current vs east and west
			if (dir <= 0 && dir <= M_PI/4 && magnitude.at<uchar>(r,c) > magnitude.at<uchar>(r,c + 1) && magnitude.at<uchar>(r,c) > magnitude.at<uchar>(r,c - 1)) {
				nms.at<uchar>(r,c) = magnitude.at<uchar>(r,c);
			}
			// check south east and north west
			else if (dir > M_PI/4 && dir <= M_PI/2 && magnitude.at<uchar>(r,c) > magnitude.at<uchar>(r + 1, c + 1) && magnitude.at<uchar>(r,c) > magnitude.at<uchar>(r - 1 ,c - 1)) {
				nms.at<uchar>(r,c) = magnitude.at<uchar>(r,c);
			}
			// check north and south
			else if (dir > M_PI/2 && dir <= (2*M_PI)/3 && magnitude.at<uchar>(r,c) > magnitude.at<uchar>(r - 1,c) && magnitude.at<uchar>(r,c) > magnitude.at<uchar>(r + 1,c)) {
				nms.at<uchar>(r,c) = magnitude.at<uchar>(r,c);
			}
			// check north east and south west
			else if (dir > (2*M_PI)/3 && dir <= M_PI && magnitude.at<uchar>(r,c) > magnitude.at<uchar>(r - 1,c + 1) && magnitude.at<uchar>(r,c) > magnitude.at<uchar>(r + 1,c - 1)) {
				nms.at<uchar>(r,c) = magnitude.at<uchar>(r,c);
			}
			else {
				nms.at<uchar>(r,c) = 0;	
			}	
		}
	}

	cv::Mat out = hysteresis(nms,t_low, t_high); 
	
  cv::imshow ( window_name_one, src);
  cv::imshow ( window_name_two, magnitude);
  cv::imshow ( window_name_three, nms);
  cv::imshow ( window_name_four, out);
	cv::waitKey(0);

	std::stringstream ss;
	ss << window_name_one << ".jpg";
  cv::imwrite ( ss.str().c_str(), src);
	ss.str("");

	ss << window_name_two << ".jpg";
  cv::imwrite ( ss.str().c_str(), magnitude);
	ss.str("");
	
	ss << window_name_three << ".jpg";
  cv::imwrite ( ss.str().c_str(), nms);
  ss.str("");
		
	ss << window_name_four << ".jpg";
  cv::imwrite ( ss.str().c_str(), out);

  return 0;
}
