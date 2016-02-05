#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cstdlib>
#include <cstdio>

#define RAD2DEG(x) (180 *(x) / M_PI)

int main( int argc, char** argv )
{
	if (argc != 2) {
		std::cout << argv[0] << "<input image>" << std::endl;
		return -1;
	}
  cv::Mat src, src_gray;
  cv::Mat grad;
  const char* window_name = "Sobel Demo - Simple Edge Detector";
  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;

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
			orientation.at<short>(r,c) = RAD2DEG(atan2( dyVal , dxVal )) + 180;
		}
	}

  // Non-maximum suppression, straightforward implementation.
  /*for (int i = 1; i < nx - 1; i++) {
  	for (int j = 1; j < ny - 1; j++) {
     	const int c = i + nx * j;
      const int nn = c - nx;
			const int ss = c + nx;
			const int ww = c + 1;
			const int ee = c - 1;
			const int nw = nn + 1;
			const int ne = nn - 1;
			const int sw = ss + 1;
			const int se = ss - 1;
 
			const float dir = (float)(fmod(atan2(after_Gy[c],
                                                 after_Gx[c]) + M_PI,
                                           M_PI) / M_PI) * 8;
 
			if (((dir <= 1 || dir > 7) && G[c] > G[ee] &&
				G[c] > G[ww]) || // 0 deg
				((dir > 1 && dir <= 3) && G[c] > G[nw] &&
 				G[c] > G[se]) || // 45 deg
				((dir > 3 && dir <= 5) && G[c] > G[nn] &&
				G[c] > G[ss]) || // 90 deg
				((dir > 5 && dir <= 7) && G[c] > G[ne] &&
				G[c] > G[sw]))   // 135 deg
 					nms[c] = G[c];
			else
 				nms[c] = 0;
		}
	}	*/
	//std::cout << mag << std::endl;	
	std::cout << orientation << std::endl;
  /// Total Gradient (approximate)
  //cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

  cv::imshow ( window_name, src);
	//cv::imshow( "orientation img", )

  cv::waitKey(0);

  return 0;
}
