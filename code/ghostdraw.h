#pragma once
#include "definitions.h"
#include "CylinderBody.h"
#include "Limbrary.h"

#define GD_DRAW 0x01
#define GD_CYL 0x02
#define GD_NOLIMBRARY 0x04
#define GD_NOBLEND 0x08

void ghostdraw(int frame, cv::Mat transform, std::vector<SkeleVideoFrame>& vidRecord, std::vector<Skeleton>& wcSkeletons, CylinderBody& cylinderBody, Limbrary& limbrary, cv::Mat draw, unsigned char options = GD_DRAW);