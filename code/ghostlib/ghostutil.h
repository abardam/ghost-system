#ifndef __GHOSTUTIL
#define __GHOSTUTIL

#include "TooN\TooN.h"

#include <array>
#include <opencv2\opencv.hpp>
#include "definitions.h"

#include <GL\glew.h>


//#define WIDTH 640
//#define HEIGHT 480
#define ON_TABLET 0

using TooN::Vector;

	double constrainAngle(double x);

	double bisect(double a, double b);

	std::array<Vector<4>, NUMJOINTS> convertArrayOfCVintoTooN(cv::Mat arr);
	
#ifdef GH_CMAPPING
	void glVertex(cv::Mat m);

	
	template<typename T, int size>
	void glVertex(cv::Vec<T,size> v){
		switch(size){
			case 2:
				glVertex2f(v(0), v(1));
				break;
			case 3:
				glVertex3f(v(0), v(1), v(2));
				break;
			case 4:
				glVertex4f(v(0), v(1), v(2), v(3));
				break;
			default:
				std::cout << "glVertex: wrong size vector!\n";
		}
	};
#endif
	
void lineAt(cv::Mat img, cv::Vec2f a, cv::Vec2f b, IMGPIXEL color);

int calcBinFromFacing(cv::Vec3f facing);

float calculateScore(cv::Mat a, cv::Mat b);


void createMask(const cv::Mat& src, cv::Mat& src_b);
void applyMask(const cv::Mat& src, const cv::Mat& mask, cv::Mat& dst, int morph_size);
void fillHoles(const cv::Mat& src, cv::Mat& dst, int morph_size);

#endif 

