#pragma once
#include <opencv2\opencv.hpp>

#include "definitions.h"
#include "texturesearch.h"
#include "cylinderprojection.h"

#define CMPC_NO_OCCLUSION 0 //oldskool way. not using limbrary
#define CMPC_BLEND_NONE 1
#define CMPC_BLEND_1 2

typedef std::pair<std::vector<cv::Vec3s>,std::vector<cv::Scalar>> PixelColorMap;

//PixelColorMap cylinderMapPixelsColor(cv::Vec3f from_a, cv::Vec3f from_b, float radius, int limbid, float facing, ScoreList scoreList, cv::Point voff, 
//									 std::vector<SkeleVideoFrame> * vidRecord, CylinderBody * cylinderBody, Limbrary * limbrary, int blendMode);

void cylinderMapPixelsColor_parallel(	cv::Vec3f from_a[NUMLIMBS], 
										cv::Vec3f from_b[NUMLIMBS], 
										float facing[NUMLIMBS], 
										ScoreList scoreList[NUMLIMBS],
										cv::Point voff[NUMLIMBS], 
										std::vector<SkeleVideoFrame> * vidRecord, 
										CylinderBody * cylinderBody, 
										Limbrary * limbrary, 
										int blendMode,
										cv::Mat fromPixels[NUMLIMBS],
										std::vector<cv::Vec3s> fromPixels_2d_v[NUMLIMBS],
										//PixelColorMap from_color[NUMLIMBS])
										cv::Mat draw);

void cylinderMapPixelsColor_parallel_orig(
	cv::Vec3f from_a[NUMLIMBS], 
	cv::Vec3f from_b[NUMLIMBS], 
	float facing[NUMLIMBS], 
	ScoreList scoreList[NUMLIMBS],
	cv::Point voff[NUMLIMBS], 
	std::vector<SkeleVideoFrame> * vidRecord, 
	CylinderBody * cylinderBody, 
	Limbrary * limbrary, 
	int blendMode,
	cv::Mat fromPixels[NUMLIMBS],
	std::vector<cv::Vec3s>& fromPixels_2d_v,
	int limits[NUMLIMBS],
	//PixelColorMap from_color[NUMLIMBS])
	cv::Mat& draw,
	cv::Mat& zBuf);


#define CP_OVER 1 //pixel not inside limb image
#define CP_OCCLUDED 2 //"blue" pixel read
#define CP_BG 3 //"white" pixel read
#define CP_GOOD 4
int colorPixel(const cv::Point2i& pt, const int& limbid, const CroppedCvMat& texture, cv::Scalar * const pixelColors);

float tempCalcFacing(int limb, Skeleton s);