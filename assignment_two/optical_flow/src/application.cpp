/*
* Matthew England
*/
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <math.h>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace cv;
using namespace std;


#define INPUT1 "INPUT IMG 1"
#define INPUT2 "INPUT IMG 2"
#define U "U vector"
#define V "V vector"
#define OUTPUT "FINAL"

/*
* Calculate the derivate of the x direction of the imagei
* kernel -1 1 applied to both
*				 -1 1

*/
Mat get_fx(Mat &src1, Mat &src2){
    Mat fx;
    Mat kernel = Mat::ones(2, 2, CV_64FC1);
    kernel.at<double>(0, 0) = -1.0;
    kernel.at<double>(1, 0) = -1.0;

    Mat dst1, dst2;
    filter2D(src1, dst1, -1, kernel);
    filter2D(src2, dst2, -1, kernel);

    fx = dst1 + dst2;
    return fx;
}

/*
* Calculate the derivate of the y direction of the image
* 
* kernel -1 -1 applied to both
*				  1  1
*/
Mat get_fy(Mat &src1, Mat &src2){
    Mat fy;
    Mat kernel = Mat::ones(2, 2, CV_64FC1);
    kernel.at<double>(0, 0) = -1.0;
    kernel.at<double>(0, 1) = -1.0;

    Mat dst1, dst2;
    filter2D(src1, dst1, -1, kernel);
    filter2D(src2, dst2, -1, kernel);

    fy = dst1 + dst2;
    return fy;
}

/*
* Calculate the derivate of the time of the image
*
* kernel 1 -1 -1 applied to src1
					 -1 -1
*
* kernel 2  1  1 apllied to src 2
*					 	1  1 
*/
Mat get_ft(Mat &src1, Mat &src2){
    Mat ft;
    Mat kernel = Mat::ones(2, 2, CV_64FC1);
    kernel = kernel.mul(-1);

    Mat dst1, dst2;
    filter2D(src1, dst1, -1, kernel);
    kernel = kernel.mul(-1);
    filter2D(src2, dst2, -1, kernel);

    ft = dst1 + dst2;
    return ft;
}

/*
* Used for summation of the partial derivatives
*/
Mat get_window_sum (Mat &m, int window_radius) {
	size_t total_window_size = (window_radius + window_radius + 1) * (window_radius + window_radius + 1);
	cv::Mat res = cv::Mat::zeros(m.rows,m.cols,CV_64FC1);
	for (int r = window_radius; r < m.rows - window_radius; r++) {
		for (int  c = window_radius; c < m.cols - window_radius; c++) {
			// traversing local window to create a sum
			size_t cnt = 0;
			double val = 0.0;
			//std::cout << cnt << std::endl;
			for (int lr = -window_radius; lr <= window_radius; lr++) {
				for (int lc = -window_radius; lc <= window_radius; lc++) {
					// valid index 
					if (((lr + r) >= 0 && (lr+r) < m.rows) && ((lc+c) >=0 && (lc+c) < m.cols )) {
						// summing the window
						++cnt;
						val += m.at<double>(lr + r,lc + c);
					}
				}
			}
			if (cnt == total_window_size)	
				res.at<double>(r,c) = val; 
			else {
				res.at<double>(r,c) *= total_window_size;
			}
		}
	}
	return res;
}

/*
* Calculates the LK optical flow on a image
* takes the input images and the desired window radius
* returns the u and v generated from the image 
*/
void calc_lucas_kanade_optical_flow(Mat &img1, Mat &img2, int window_radius, Mat &u, Mat &v){

		// Calculate the first derivatives of the image for x,y, and time
    Mat fx = get_fx(img1, img2);
    Mat fy = get_fy(img1, img2);
    Mat ft = get_ft(img1, img2);

		// Calculate the partial derivates
    Mat fx2 = fx.mul(fx); //Ix^2
    Mat fy2 = fy.mul(fy); //Iy^2
    Mat fxfy = fx.mul(fy); // Ixy
    Mat fxft = fx.mul(ft); // Ixt
    Mat fyft = fy.mul(ft); // Iyt



    // Get the sum of the windows
		Mat sum_fx2 = get_window_sum(fx2,window_radius); // sum Ix^2
    Mat sum_fy2 = get_window_sum(fy2,window_radius); // sum Iy^2
    Mat sum_fxft = get_window_sum(fxft,window_radius); // sum Ixy
    Mat sum_fxfy = get_window_sum(fxfy,window_radius); // sum Ixt
    Mat sum_fyft = get_window_sum(fyft,window_radius); // sum Iyt

		// Does the least squares method	
    Mat tmp = sum_fx2.mul(sum_fy2) - sum_fxfy.mul(sum_fxfy);
   	// Calculate the U and V
		// solve for U first 
		u = sum_fxfy.mul(sum_fyft) - sum_fy2.mul(sum_fxft);
		// solve for V second
    v = sum_fxft.mul(sum_fxfy) - sum_fx2.mul(sum_fyft);
		// normalize the U and V matrix
    divide(u, tmp, u);
    divide(v, tmp, v);
}



int main(int argc, char** argv){
	if (argc != 4) {
		std::cout << argv[0] << " <input image one > <input image two> <window radius>" << std::endl;
		return -1;
	}
	Mat img1;
	Mat img2;
	Mat in = imread(argv[1]);
	Mat out = in.clone();
	  /// Convert it to gray to 1 channel image
  cv::cvtColor( in, img1, cv::COLOR_RGB2GRAY );
	
	if (!img1.data) {
	std::cout << "NO IMAGE 1 TO READ" << std::endl;
	return -1;
	}
  Mat in2 = imread(argv[2]);
	if (!in2.data) {
	std::cout << "NO IMAGE 2 TO READ" << std::endl;
	return -1;
	}
	 /// Convert it to gray to 1 channel image
  cv::cvtColor( in2, img2, cv::COLOR_RGB2GRAY );


	int window_radius = atoi(argv[3]);
	// convert to 64 bit gray scale
  img1.convertTo(img1, CV_64FC1, 1.0/255, 0);
  img2.convertTo(img2, CV_64FC1, 1.0/255, 0);


  Mat u = Mat::zeros(img1.rows, img1.cols, CV_64FC1);
  Mat v = Mat::zeros(img1.rows, img1.cols, CV_64FC1);
  
	calc_lucas_kanade_optical_flow(img1, img2, window_radius,u, v);

	 /*Create the Vector Arrows on the input image*/
   IplImage temp_u = u;
   IplImage temp_v = v;
    double l_max = -10;
    for (int y = 0; y < img1.rows; y+=10) // First iteration, to compute the maximum l (longest flow)
    {
        for (int x = 0; x < img1.cols; x+=10)
        {
            double dx = cvGetReal2D(&temp_u, y, x);  // Gets X component of the flow
            double dy = cvGetReal2D(&temp_v, y, x);  // Gets Y component of the flow
            double l = sqrt(dx*dx + dy*dy); // This function sets a basic threshold for drawing on the image

            if(l>l_max) l_max = l;
        }
    }

   for (int y = 0; y < img1.rows; y+=10)
   {
   	for (int x = 0; x < img1.cols; x+=10)
   	 {
        double dx = cvGetReal2D(&temp_u, y, x); // Gets X component of the flow
        double dy = cvGetReal2D(&temp_v, y, x); // Gets Y component of the flow

        Point p = cvPoint(x, y);

        double l = sqrt(dx*dx + dy*dy); // This function sets a basic threshold for drawing on the image
        if (l > 0)
        {
            double spinSize = 5.0 * l/l_max; // Factor to normalise the size of the spin depeding on the length of the arrow

            CvPoint p2 = cvPoint(p.x + (int)(dx), p.y + (int)(dy));
            cv::line(out, p, p2, CV_RGB(0,255,0), 1, CV_AA);

            double angle;  // Draws the spin of the arrow
            angle = atan2( (double) p.y - p2.y, (double) p.x - p2.x );

            p.x = (int) (p2.x + spinSize * cos(angle + 3.1416 / 4));
            p.y = (int) (p2.y + spinSize * sin(angle + 3.1416 / 4));
            cv::line( out, p, p2, CV_RGB(0,255,0), 1, CV_AA, 0 );

            p.x = (int) (p2.x + spinSize * cos(angle - 3.1416 / 4));
            p.y = (int) (p2.y + spinSize * sin(angle - 3.1416 / 4));
            cv::line( out, p, p2, CV_RGB(0,255,0), 1, CV_AA, 0 );

        }
   }
	}

	cv::namedWindow(INPUT1, 	 cv::WINDOW_AUTOSIZE);
	cv::namedWindow(INPUT2,	 cv::WINDOW_AUTOSIZE);
	cv::namedWindow(U, cv::WINDOW_AUTOSIZE);
	cv::namedWindow(V,	 cv::WINDOW_AUTOSIZE);
	cv::namedWindow(OUTPUT,	 cv::WINDOW_AUTOSIZE);
	// display to the user
	cv::imshow(INPUT1, in);	
	cv::imshow(INPUT2, in2);
	cv::imshow(U, u);	
	cv::imshow(V, v);
	cv::imshow(OUTPUT,out);

	waitKey(0);
	// Write the files created to file
	std::stringstream ss;
	ss << INPUT1 << ".jpg";
	cv::imwrite(ss.str().c_str(), in);	
	ss.str("");	
	ss << INPUT2 << ".jpg";
	cv::imwrite(ss.str().c_str(), in2);	
	ss.str("");	
	ss << U << ".jpg";
	cv::imwrite(ss.str().c_str(), u);	
	ss.str("");	
	ss << V << ".jpg";
	cv::imwrite(ss.str().c_str(), v);	
	ss.str("");	
	ss << OUTPUT << ".jpg";
	cv::imwrite(ss.str().c_str(), out);	
	ss.str("");	

  return 0;
}
