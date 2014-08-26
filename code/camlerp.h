#pragma once
#include <opencv2\opencv.hpp>

struct LerpCorners{
	cv::Vec3f topLeft, topRight, botLeft, botRight;
};

void initLerp(int width, int height);

LerpCorners generateLerpCorners(const cv::Rect& boundingBox);
cv::Vec3f lerpPoint(const int& x, const int& y, const cv::Rect& boundingBox, const LerpCorners& lc);

extern cv::Rect lerpBoundingBox;
extern LerpCorners lerpCorners;