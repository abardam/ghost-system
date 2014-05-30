#pragma once

#include <opencv2\opencv.hpp>

#include "definitions.h"
#include "CylinderBody.h"

void initAndLoad(cv::Mat mse3CfW, cv::Mat K2P, std::vector<SkeleVideoFrame> * vidRecord,std::vector<Skeleton> * wcSkeletons, std::string path = "video/", bool loadRGB = true);
void buildCylinderBody(std::vector<SkeleVideoFrame> * vidRecord, CylinderBody * cb);
void calculateWorldCoordinateSkeletons(cv::Mat K2P, std::vector<SkeleVideoFrame> * vidRecord, std::vector<Skeleton> * outputSkeletons);