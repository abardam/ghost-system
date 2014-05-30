#include "motionsmooth.h"
#include "ghostutil.h"

std::vector<double> calc_velocity(const std::vector<double>& x){
	std::vector<double> v(x.size());

	for(int i=0;i<x.size();++i){
		int j=i-WINDOW;
		j=j<0?0:j;
		int k=i+WINDOW;
		k=k>=x.size()?x.size()-1:k;

		v[i] = (x[k] - x[j]) / ((k-j)/20.);
	}

	return v;
}


std::vector<double> exp_smooth(const std::vector<double>& x){
	std::vector<double> s(x.size());
	s[0] = x[0];
	for(int i=1;i<x.size();++i){
		s[i] = EXP_A * x[i-1] + (1-EXP_A) * s[i-1];
	}
	return s;
}

cv::Mat exp_smooth(const cv::Mat x, double exp_a){
	cv::Mat a(x.size(), CV_32F);
	for(int r=0;r<x.rows;++r){
		a.at<float>(r,0) = x.at<float>(r,0);
		for(int c=1;c<x.cols;++c){
			a.at<float>(r,c) = exp_a * x.at<float>(r,c-1) + (1-exp_a) * a.at<float>(r,c-1);
		}
	}
	return a;
}

void smoothKinectPoints(std::vector<Skeleton>& wcSkeletons, double exp_a){

	for(int i=0;i<NUMLIMBS;++i){
		cv::Mat lpts(4, wcSkeletons.size(), CV_32F);

		for(int j=0;j<wcSkeletons.size();++j){
			for(int k=0;k<4;++k){
				lpts.at<float>(k,j) = wcSkeletons[j].points.at<float>(k,i);
			}
		}

		lpts = exp_smooth(lpts, exp_a);

		
		for(int j=0;j<wcSkeletons.size();++j){
			for(int k=0;k<4;++k){
				wcSkeletons[j].points.at<float>(k,i) = lpts.at<float>(k,j);
			}
		}
	}

}