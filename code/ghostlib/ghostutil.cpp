
#include "ghostutil.h"
#include "TooN\TooN.h"

#include "KinectManager.h"

#include "opencv2\opencv.hpp"
#include <array>
#include <GL\glew.h>

using TooN::Vector;
	
	double constrainAngle(double x){
		x = fmod(x,2*CV_PI);
		if (x < 0)
			x += 2*CV_PI;
		return x;
	}

	double bisect(double a, double b){
		return (constrainAngle(a) + constrainAngle(b))/2;
	}
#ifdef GH_CMAPPING
	void glVertex(cv::Mat m){

		int size = m.total();

		switch(size){
			case 2:
				glVertex2f(m.at<float>(0), m.at<float>(1));
				break;
			case 3:
				glVertex3f(m.at<float>(0), m.at<float>(1), m.at<float>(2));
				break;
			case 4:
				glVertex4f(m.at<float>(0), m.at<float>(1), m.at<float>(2), m.at<float>(3));
				break;
			default:
				std::cout << "glVertex: wrong size matrix!\n";
		}
	}
	
#endif

	std::array<Vector<4>, NUMJOINTS> convertArrayOfCVintoTooN(cv::Mat arr){
		std::array<Vector<4>, NUMJOINTS> retval;
		for(int i=0;i<NUMJOINTS;++i){
			for(int j=0;j<4;++j)
				retval[i][j] = arr.at<float>(j,i);
		}

		return retval;
	}

	
int calcBinFromFacing(cv::Vec3f facing){
	//first project facing to the horizontal (a=0, b=1, c=0, d=0) plane

	cv::Vec3f vert(0,1,0);

	float fdot = facing.dot(vert);
	cv::Vec3f fproj = fdot * vert;

	cv::Vec3f frej = facing - fproj;

	//frej should have y component of 0 now...

	float angle = atan2f(frej(2), frej(0));

	angle += CV_PI;
	//angle now goes from 0 to 2pi
	//bin it

	float x = NUMBINS/(2*CV_PI);

	int bin = angle * x;

	return bin;
}



//drawing


void lineAt(cv::Mat img, cv::Vec2f a, cv::Vec2f b, IMGPIXEL color){

	//clamp a and b to the edge of the img
	float lambda;
	for(int i=0;i<=1;++i){
		lambda = 0;
		if(a[i] < 0){
			lambda = (0 - a[i])/(b[i]-a[i]);
		}else if(a[i] > img.size[1-i]-1){
			lambda = (img.size[1-i]-1-a[i])/(b[i]-a[i]);
		}
		a = a + lambda*(b-a);
		
		lambda = 0;
		if(b[i] < 0){
			lambda = (0 - b[i])/(a[i]-b[i]);
		}else if(b[i] > img.size[1-i]-1){
			lambda = (img.size[1-i]-1-b[i])/(a[i]-b[i]);
		}
		b = b + lambda*(a-b);
	}

	int x0 = a[0];
	int x1 = b[0];
	int y0 = a[1];
	int y1 = b[1];
	int dx = abs(x1-x0);
	int dy = abs(y1-y0);
	int sx, sy;
	if(x0 < x1) sx = 1; else sx = -1;
	if(y0 < y1) sy = 1; else sy = -1;
	int err = dx-dy;
 
	int e2;

	while(true)
	{
		if(y0 < 0 || y0 >= img.rows ||
			x0 < 0 || x0 >= img.cols) return;
		img.at<IMGPIXEL>(y0,x0) = color;
		if(x0 == x1 && y0 == y1) return;
		e2 = 2*err;
		if(e2 > -dy)
		{
			err = err - dy;
			x0 = x0 + sx;
		}
		if(x0 == x1 && y0 == y1)
		{
			img.at<IMGPIXEL>(y0,x0) = color;
			return;
		}
		if(e2 < dx){
			err = err + dx;
			y0 = y0 + sy;
		}
	}
};


float calculateScore(cv::Mat a, cv::Mat b){
	float bst=0;	
	for(int j=0; j<NUMJOINTS; ++j){
		bst += cv::norm(b.col(j)-a.col(j));
	}

	return bst;
}

void createMask(const cv::Mat& src, cv::Mat& src_b){
	src_b.create(src.size(), CV_8U);
	for (int i = 0; i<src.rows*src.cols; ++i){
		src_b.ptr<unsigned char>()[i] = src.ptr<cv::Vec3b>()[i] == cv::Vec3b(255, 255, 255) ? 0 : 255;
	}
}



void applyMask(const cv::Mat& src, const cv::Mat& mask, cv::Mat& dst, int morph_size){
	dst.create(src.size(), src.type());
	for (int r = 0; r<src.rows; ++r){
		for (int c = 0; c<src.cols; ++c){
			if (mask.ptr<unsigned char>(r)[c] == 255){
				cv::Vec3b color = src.ptr<cv::Vec3b>(r)[c];
				if (color == cv::Vec3b(255, 255, 255)){
					int c1 = 0, c2 = 0, c3 = 0;
					int count = 0;
					int occludedcount = 0;
					//look for colors in a range around the pixel
					for (int j = -morph_size; j <= morph_size; ++j){
						for (int k = -morph_size; k <= morph_size; ++k){
							if (r + j<0 || r + j >= src.rows || c + k<0 || c + k >= src.cols) continue;
							cv::Vec3b representative_color = src.ptr<cv::Vec3b>(r + j)[c + k];
							if(representative_color == cv::Vec3b(255,0,0)){
								++occludedcount;
							}
							else if (representative_color != cv::Vec3b(255, 255, 255)){
								c1 += representative_color(0);
								c2 += representative_color(1);
								c3 += representative_color(2);
								++count;
							}
						}
					}

					if(occludedcount > count){
						c1 = 255;
						c2 = 0;
						c3 = 0;
						count = 1;
					}

					if (count == 0) count = 1;

					cv::Vec3b new_color(c1 / count,
						c2 / count,
						c3 / count);

					dst.ptr<cv::Vec3b>(r)[c] = new_color;

				}
				else{
					dst.ptr<cv::Vec3b>(r)[c] = color;
				}
			}
			else{
				dst.ptr<cv::Vec3b>(r)[c] = cv::Vec3b(255, 255, 255);
			}
		}
	}
}

void fillHoles(const cv::Mat& src, cv::Mat& dst, int morph_size){

	cv::Mat mask;
	createMask(src, mask);

	cv::Mat mask_dst;

	cv::morphologyEx(mask, mask_dst, CV_MOP_CLOSE,
		cv::getStructuringElement(CV_SHAPE_ELLIPSE, cv::Size(2 * morph_size + 1, 2 * morph_size + 1), cv::Point(morph_size, morph_size)));

	applyMask(src, mask_dst, dst, morph_size);

}


void removeIslands(const cv::Mat& src, cv::Mat& dst, int morph_size){

	cv::Mat mask;
	createMask(src, mask);

	cv::Mat mask_dst;

	cv::morphologyEx(mask, mask_dst, CV_MOP_OPEN,
		cv::getStructuringElement(CV_SHAPE_ELLIPSE, cv::Size(2 * morph_size + 1, 2 * morph_size + 1), cv::Point(morph_size, morph_size)));

	applyMask(src, mask_dst, dst, morph_size);
}