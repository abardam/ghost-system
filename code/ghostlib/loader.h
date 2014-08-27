#pragma once


#include <vector>
#include <array>

#include "definitions.h"
#include "CylinderBody.h"
#include "Skeleton.h"

#include <opencv2\opencv.hpp>
#include <TooN\TooN.h>
#include "TooN\se3.h"

void initLoader();
std::vector<bool> LoadVideo(cv::Mat cam2World, cv::Mat K2P, std::vector<SkeleVideoFrame> * vidRecord, std::vector<Skeleton> * wcSkeletons, std::string path = "video/", bool loadRGB = true);
void SaveVideo(std::vector<SkeleVideoFrame> * vidRecord, std::string path = "video/");

//IMGPIXEL getColorAtPartAndPixel(int frame, int part, cv::Vec4f pixel, cv::Vec2f * pixelLoc = 0);

void LoadWorldCoordinateSkeletons(std::vector<Skeleton>& wcSkeletons, std::string path);
