
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
	

	std::array<Vector<4>, NUMJOINTS> convertArrayOfCVintoTooN(cv::Mat arr){
		std::array<Vector<4>, NUMJOINTS> retval;
		for(int i=0;i<NUMJOINTS;++i){
			for(int j=0;j<4;++j)
				retval[i][j] = arr.at<float>(j,i);
		}

		return retval;
	}