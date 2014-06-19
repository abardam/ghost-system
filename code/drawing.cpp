#include "drawing.h"
#include "definitions.h"

std::vector<cv::Point> cloudOutline(cv::Mat _2D, int wd, int ht){

	std::vector<cv::Point> outline;

	std::vector<int> up(wd, ht);
	std::vector<int> dn(wd, -1);
	std::vector<int> lt(ht, wd);
	std::vector<int> rt(ht, -1);

	for(int c=0; c<_2D.cols; ++c){
		cv::Point pt2d = cv::Point(_2D.at<float>(0,c),_2D.at<float>(1,c));

		if(pt2d.x >= 0 && pt2d.x < wd && pt2d.y >= 0 && pt2d.y < ht)
		{
			up[pt2d.x] = std::min(up[pt2d.x], pt2d.y);
			dn[pt2d.x] = std::max(dn[pt2d.x], pt2d.y);
			lt[pt2d.y] = std::min(lt[pt2d.y], pt2d.x);
			rt[pt2d.y] = std::max(rt[pt2d.y], pt2d.x);
		}
	}

	for(int x=0; x<wd; ++x){
		if(up[x] >= 0 && up[x] < ht)
			outline.push_back(cv::Point(x, up[x]));
		if(dn[x] >= 0 && dn[x] < ht)
			outline.push_back(cv::Point(x, dn[x]));
	}
	
	for(int y=0; y<ht; ++y){
		if(lt[y] >= 0 && lt[y] < wd)
			outline.push_back(cv::Point(lt[y], y));
		if(rt[y] >= 0 && rt[y] < wd)
			outline.push_back(cv::Point(rt[y], y));
	}

	return outline;
}

//changed from Mat.at
void cvDrawPoint(cv::Mat im, cv::Point p, cv::Scalar color){
	if(CLAMP_SIZE(p.x, p.y, im.cols, im.rows)){
		int nc = im.channels();
		if(nc == 1){
			im.ptr<unsigned char>(p.y)[p.x] = color(0);
		}else if(nc == 2){
			im.ptr<cv::Vec2b>(p.y)[p.x] = cv::Vec2b(color(0), color(1));
		}else if(nc == 3){
			im.ptr<cv::Vec3b>(p.y)[p.x] = cv::Vec3b(color(0), color(1), color(2));
		}else if(nc == 4){
			im.ptr<cv::Vec4b>(p.y)[p.x] = cv::Vec4b(color(0), color(1), color(2), color(3));
		}
	}
}
