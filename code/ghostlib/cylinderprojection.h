#pragma once

#include "bodybuild.h"

//#define MAXDEPTH 20000
#define MAXDEPTH 8000
#define FLOAT_TO_DEPTH 1000
#define WIDTH 640
#define HEIGHT 480


typedef std::pair<std::vector<cv::Point>,std::vector<cv::Point>> PixelMap;

cv::Mat cylinder_to_pts(cv::Vec3f a, cv::Vec3f b, float radius, cv::Point voff, PixelPolygon * p, std::vector<cv::Vec3f> * fromPixels, std::vector<cv::Vec3s> * fromPixels_2d_v);
cv::Mat cylinder_to_pts(cv::Vec3f a, cv::Vec3f b, float radius, cv::Point voff, std::vector<cv::Vec3f> * fromPixels, std::vector<cv::Vec3s> * fromPixels_2d_v);
//copy of cylinder_to_pts that involves a cv::Rect instead of a PixelPolygon
cv::Mat cylinder_to_pts(cv::Vec3f a_, cv::Vec3f b_, float radius, cv::Point voff, cv::Rect * r, std::vector<cv::Vec3f> * fromPixels, std::vector<cv::Vec3s> * fromPixels_2d_v);

cv::Mat pts_to_zBuffer(cv::Mat cylPts, cv::Point voff, cv::Point offset, unsigned int width, unsigned int height);

//returns a 4x4 matrix transforming from cylinder 1 to cylinder 2
cv::Mat cylinderFacingTransform(cv::Vec3f a1, cv::Vec3f b1, float f1, cv::Vec3f a2, cv::Vec3f b2, float f2, float radiusModifier = 1);

//same but with cv::Mat
cv::Mat cylinderFacingTransform(cv::Mat a1, cv::Mat b1, float f1, cv::Mat a2, cv::Mat b2, float f2);

//updated version of ^ that uses SVD
cv::Mat cylinderFacingTransform2(cv::Vec3f a1, cv::Vec3f b1, float f1, cv::Vec3f a2, cv::Vec3f b2, float f2);

//given 2 cylinders, assuming same radius, maps pixels from one to the other
PixelMap cylinderMapPixels(cv::Vec3f from_a, cv::Vec3f from_b, cv::Vec3f to_a, cv::Vec3f to_b, float radius, CroppedCvMat * fromMat, CroppedCvMat * toMat, cv::Size captureWindow, cv::Point captureOffset);


cv::Point2i mapPixel(cv::Vec3f pixLoc, cv::Point2i pixOffset, cv::Vec3f from_a, cv::Vec3f from_b, float from_facing, cv::Vec3f to_a, cv::Vec3f to_b, float to_facing);


float tempCalcFacing(int limb, Skeleton s);