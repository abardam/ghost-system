#pragma once
#include "definitions.h"
#include "CylinderBody.h"
#include "Limbrary.h"
#include "texturesearch.h"


#define GD_DRAW 0x01
#define GD_CYL 0x02
#define GD_NOLIMBRARY 0x04
#define GD_NOBLEND 0x08
#define GD_NOWEIGHT 0x10

//void ghostdraw(int frame, cv::Mat transform, std::vector<SkeleVideoFrame>& vidRecord, std::vector<Skeleton>& wcSkeletons, CylinderBody& cylinderBody, Limbrary& limbrary, cv::Mat draw, unsigned char options = GD_DRAW);
void ghostdraw_parallel
	(int frame, cv::Mat transform, std::vector<SkeleVideoFrame>& vidRecord, std::vector<Skeleton>& wcSkeletons, CylinderBody& cylinderBody, Limbrary& limbrary, cv::Mat& draw, cv::Mat& zBuf, unsigned char options = GD_DRAW, ScoreList * scoreListOut = 0);

void ghostdraw_prep(int frame, const cv::Mat& transform, int texSearchDepth, int wtType, const std::vector<SkeleVideoFrame>& vidRecord, const std::vector<Skeleton>& wcSkeletons, const CylinderBody& cylinderBody, const Limbrary& limbrary, cv::Vec3f a[NUMLIMBS], cv::Vec3f b[NUMLIMBS], float facing[NUMLIMBS], ScoreList scoreList[NUMLIMBS], cv::Point offsets[NUMLIMBS], cv::Mat fromPixels[NUMLIMBS], std::vector<cv::Vec3s>& fromPixels_2d_v, int limits[NUMLIMBS]);