#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <math.h>
#include <fstream>
#include <iostream>

using namespace cv;
using namespace std;


#define INPUT1 "INPUT IMG 1"
#define INPUT2 "INPUT IMG 2"
#define U "U vector"
#define V "V vector"
#define OUTPUT "FINAL"

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


Mat get_window_sum (Mat &m, int window_radius) {
	cv::Mat res = cv::Mat::zeros(m.rows,m.cols,CV_64FC1);
	for (int r = 0; r < m.rows; r++) {
		for (int  c = 0; c < m.cols; c++) {
			// traversing local window to create a sum
			for (int lr = -window_radius; lr <= window_radius; lr++) {
				for (int lc = -window_radius; lc <= window_radius; lc++) {
					// valid index 
					if (((lr + r) >= 0 && (lr+r) < m.rows) && ((lc+c) >=0 && (lc+c) < m.cols )) {
						// summing the window
						res.at<double>(r,c) += m.at<double>(lr,lc);
					}	
				}
			}
		}
	}
	return res;
}



bool isInsideImage(int y, int x, Mat &m){
    int width = m.cols;
    int height = m.rows;
    if(x >= 0 && x < width && y >= 0 && y < height) return true;
    else return false;
}

double get_Sum9(Mat &m, int y, int x){
    if(x < 0 || x >= m.cols) return 0;
    if(y < 0 || y >= m.rows) return 0;

    double val = 0.0;
    int tmp = 0;

    if(isInsideImage(y - 1, x - 1, m)){
        ++ tmp;
        val += m.at<double>(y - 1, x - 1);
    }
    if(isInsideImage(y - 1, x, m)){
        ++ tmp;
        val += m.at<double>(y - 1, x);
    }
    if(isInsideImage(y - 1, x + 1, m)){
        ++ tmp;
        val += m.at<double>(y - 1, x + 1);
    }
    if(isInsideImage(y, x - 1, m)){
        ++ tmp;
        val += m.at<double>(y, x - 1);
    }
    if(isInsideImage(y, x, m)){
        ++ tmp;
        val += m.at<double>(y, x);
    }
    if(isInsideImage(y, x + 1, m)){
        ++ tmp;
        val += m.at<double>(y, x + 1);
    }
    if(isInsideImage(y + 1, x - 1, m)){
        ++ tmp;
        val += m.at<double>(y + 1, x - 1);
    }
    if(isInsideImage(y + 1, x, m)){
        ++ tmp;
        val += m.at<double>(y + 1, x);
    }
    if(isInsideImage(y + 1, x + 1, m)){
        ++ tmp;
        val += m.at<double>(y + 1, x + 1);
    }
    if(tmp == 9) return val;
    else return m.at<double>(y, x) * 9;
}

Mat get_Sum9_Mat(Mat &m){
    Mat res = Mat::zeros(m.rows, m.cols, CV_64FC1);
    for(int i = 1; i < m.rows - 1; i++){
        for(int j = 1; j < m.cols - 1; j++){
            res.at<double>(i, j) = get_Sum9(m, i, j);
        }
    }
    return res;
}

void saveMat(Mat &M, string s){
    s += ".txt";
    FILE *pOut = fopen(s.c_str(), "w+");
    for(int i=0; i<M.rows; i++){
        for(int j=0; j<M.cols; j++){
            fprintf(pOut, "%lf", M.at<double>(i, j));
            if(j == M.cols - 1) fprintf(pOut, "\n");
            else fprintf(pOut, " ");
        }
    }
    fclose(pOut);
}

void getLucasKanadeOpticalFlow(Mat &img1, Mat &img2, int window_radius, Mat &u, Mat &v){

    Mat fx = get_fx(img1, img2);
    Mat fy = get_fy(img1, img2);
    Mat ft = get_ft(img1, img2);

    Mat fx2 = fx.mul(fx);
    Mat fy2 = fy.mul(fy);
    Mat fxfy = fx.mul(fy);
    Mat fxft = fx.mul(ft);
    Mat fyft = fy.mul(ft);

    Mat sumfx2 = get_Sum9_Mat(fx2);
    Mat sumfy2 = get_Sum9_Mat(fy2);
    Mat sumfxft = get_Sum9_Mat(fxft);
    Mat sumfxfy = get_Sum9_Mat(fxfy);
    Mat sumfyft = get_Sum9_Mat(fyft);

    /*Mat sumfx2 = get_window_sum(fx2,window_radius);
    Mat sumfy2 = get_window_sum(fy2,window_radius);
    Mat sumfxft = get_window_sum(fxft,window_radius);
    Mat sumfxfy = get_window_sum(fxfy,window_radius);
    Mat sumfyft = get_window_sum(fyft,window_radius);*/

    Mat tmp = sumfx2.mul(sumfy2) - sumfxfy.mul(sumfxfy);
    u = sumfxfy.mul(sumfyft) - sumfy2.mul(sumfxft);
    v = sumfxft.mul(sumfxfy) - sumfx2.mul(sumfyft);
    divide(u, tmp, u);
    divide(v, tmp, v);
}



int main(int argc, char** argv){
	if (argc != 4) {
		std::cout << argv[0] << " <input image one > <input image two> <window radius>" << std::endl;
		return -1;
	}

    Mat img1 = imread(argv[1], 0);
    Mat out = img1.clone();
	if (!img1.data) {
	std::cout << "NO IMAGE 1 TO READ" << std::endl;
	return -1;
	}
    Mat img2 = imread(argv[2], 0);
	if (!img2.data) {
	std::cout << "NO IMAGE 2 TO READ" << std::endl;
	return -1;
	}
	int window_radius = atoi(argv[3]);
	// convert to 64 bit gray scale
    img1.convertTo(img1, CV_64FC1, 1.0/255, 0);
    img2.convertTo(img2, CV_64FC1, 1.0/255, 0);


    Mat u = Mat::zeros(img1.rows, img1.cols, CV_64FC1);
    Mat v = Mat::zeros(img1.rows, img1.cols, CV_64FC1);
    //Mat out = img1.clone();
	

    getLucasKanadeOpticalFlow(img1, img2, window_radius,u, v);

	cv::namedWindow(INPUT1, 	 cv::WINDOW_AUTOSIZE);
	cv::namedWindow(INPUT2,	 cv::WINDOW_AUTOSIZE);
	cv::namedWindow(U, cv::WINDOW_AUTOSIZE);
	cv::namedWindow(V,	 cv::WINDOW_AUTOSIZE);
	cv::namedWindow(OUTPUT,	 cv::WINDOW_AUTOSIZE);

   IplImage temp_u = u;
   IplImage temp_v = v;
    double l_max = -10;
    for (int y = 0; y < img1.rows; y+=10)                                                           // First iteration, to compute the maximum l (longest flow)
    {
        for (int x = 0; x < img1.cols; x+=10)
        {
            double dx = cvGetReal2D(&temp_v, y, x);                                                        // Gets X component of the flow
            double dy = cvGetReal2D(&temp_u, y, x);                                                        // Gets Y component of the flow

            //CvPoint p = cvPoint(x, y);

            double l = sqrt(dx*dx + dy*dy);                                                             // This function sets a basic threshold for drawing on the image

            if(l>l_max) l_max = l;
        }
    }

   for (int y = 0; y < img1.rows; y+=10)
   {
   	for (int x = 0; x < img1.rows; x+=10)
   	 {
        double dx = cvGetReal2D(&temp_v, y, x);                                                        // Gets X component of the flow
        double dy = cvGetReal2D(&temp_u, y, x);                                                        // Gets Y component of the flow

        Point p = cvPoint(x, y);

        double l = sqrt(dx*dx + dy*dy);                                                             // This function sets a basic threshold for drawing on the image
        if (l > 0)
        {
            double spinSize = 5.0 * l/l_max;                                                        // Factor to normalise the size of the spin depeding on the length of the arrow

            CvPoint p2 = cvPoint(p.x + (int)(dx), p.y + (int)(dy));
            cv::line(out, p, p2, CV_RGB(0,255,0), 1, CV_AA);

            double angle;                                                                           // Draws the spin of the arrow
            angle = atan2( (double) p.y - p2.y, (double) p.x - p2.x );

            p.x = (int) (p2.x + spinSize * cos(angle + 3.1416 / 4));
            p.y = (int) (p2.y + spinSize * sin(angle + 3.1416 / 4));
            cv::line( out, p, p2, CV_RGB(0,255,0), 1, CV_AA, 0 );

            p.x = (int) (p2.x + spinSize * cos(angle - 3.1416 / 4));
            p.y = (int) (p2.y + spinSize * sin(angle - 3.1416 / 4));
	    //cv::arrowedLine(out,p,p2, CV_RGB(0,255,0), 2, 1, CV_AA, 0 );
            cv::line( out, p, p2, CV_RGB(0,255,0), 1, CV_AA, 0 );

        }
   }
}
	
	cv::imshow(INPUT1, img1);	
	cv::imshow(INPUT2, img2);
	cv::imshow(U, u);	
	cv::imshow(V, v);
	cv::imshow(OUTPUT,out);

	

	 waitKey(0);

    return 0;
}
