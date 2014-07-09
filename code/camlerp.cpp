#include <opencv2\opencv.hpp>

#include "camlerp.h"
#include "cvutil.h"
#include "ghostcam.h"

cv::Rect lerpBoundingBox;
LerpCorners lerpCorners;

void initLerp(int width, int height){
	lerpBoundingBox = cv::Rect(0,0,width,height);
	lerpCorners = generateLerpCorners(lerpBoundingBox);
}

LerpCorners generateLerpCorners(const cv::Rect& boundingBox){
	LerpCorners lc;

	std::vector<cv::Vec3f> lerpVec;
	for(int _y=0;_y<=boundingBox.height;_y+=boundingBox.height){
		int y = _y + boundingBox.y;

		for(int _x=0;_x<=boundingBox.width;_x+=boundingBox.width){
			int x = _x + boundingBox.x;

			cv::Vec3f p2d(x,y,1);
			cv::Vec3f p2d_ray = mat_to_vec3(getInvCameraMatrix() * cv::Mat(p2d));

			lerpVec.push_back(p2d_ray);
		}
	}

	lc.topLeft = lerpVec[0];
	lc.topRight = lerpVec[1];
	lc.botLeft = lerpVec[2];
	lc.botRight = lerpVec[3];
	return lc;
}

cv::Vec3f lerpPoint(const int& x, const int& y, const cv::Rect& boundingBox, const LerpCorners& lc){
	return cv::Vec3f(((boundingBox.width-x+boundingBox.x+0.0f)/boundingBox.width) * lc.topLeft(0) + ((x-boundingBox.x+0.0f)/boundingBox.width) * lc.topRight(0),
			((boundingBox.height-y+boundingBox.y+0.0f)/boundingBox.height) * lc.topLeft(1) + ((y-boundingBox.y+0.0f)/boundingBox.height) * lc.botLeft(1), 1);
}