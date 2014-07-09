
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
