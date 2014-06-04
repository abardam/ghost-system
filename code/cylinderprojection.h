#pragma once

#include "bodybuild.h"
#include "Limbrary.h"
#include "texturesearch.h"

#define MAXDEPTH 20000
#define FLOAT_TO_DEPTH 1000

typedef std::pair<std::vector<cv::Point>,std::vector<cv::Point>> PixelMap;
typedef std::pair<std::vector<cv::Vec3s>,std::vector<cv::Scalar>> PixelColorMap;

cv::Mat cylinder_to_pts(cv::Vec3f a, cv::Vec3f b, float radius, cv::Point voff, PixelPolygon * p, std::vector<cv::Vec3f> * fromPixels, std::vector<cv::Vec3s> * fromPixels_2d_v);
cv::Mat pts_to_zBuffer(cv::Mat cylPts, cv::Point voff, cv::Point offset, unsigned int width, unsigned int height);

//returns a 4x4 matrix transforming from cylinder 1 to cylinder 2
cv::Mat cylinderFacingTransform(cv::Vec3f a1, cv::Vec3f b1, float f1, cv::Vec3f a2, cv::Vec3f b2, float f2);

//given 2 cylinders, assuming same radius, maps pixels from one to the other
PixelMap cylinderMapPixels(cv::Vec3f from_a, cv::Vec3f from_b, cv::Vec3f to_a, cv::Vec3f to_b, float radius, CroppedCvMat * fromMat, CroppedCvMat * toMat, cv::Size captureWindow, cv::Point captureOffset);


#define CMPC_NO_OCCLUSION 0 //oldskool way. not using limbrary
#define CMPC_BLEND_NONE 1
#define CMPC_BLEND_1 2
PixelColorMap cylinderMapPixelsColor(cv::Vec3f from_a, cv::Vec3f from_b, float radius, int limbid, float facing, ScoreList scoreList, cv::Point voff, 
									 std::vector<SkeleVideoFrame> * vidRecord, CylinderBody * cylinderBody, Limbrary * limbrary, int blendMode);

cv::Point2i mapPixel(cv::Vec3f pixLoc, cv::Point2i pixOffset, cv::Vec3f from_a, cv::Vec3f from_b, float from_facing, cv::Vec3f to_a, cv::Vec3f to_b, float to_facing);

#define CP_OVER 1 //pixel not inside limb image
#define CP_OCCLUDED 2 //"blue" pixel read
#define CP_BG 3 //"white" pixel read
#define CP_GOOD 4
int colorPixel(cv::Point2i pt, int limbid, CroppedCvMat texture, cv::Scalar * pixelColors);

float tempCalcFacing(int limb, Skeleton s);