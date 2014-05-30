#pragma once
#include <list>

#include "definitions.h"
#include "Limbrary.h"

typedef std::list<std::pair<int, float>> ScoreList;

cv::Mat normalizeSkeleton(cv::Mat skel);
ScoreList sortFrames(Skeleton s, std::vector<SkeleVideoFrame> * vidRecord, Limbrary * limbrary, unsigned int limbid = NUMLIMBS, int limit = -1, bool sort = true);
cv::Mat limbbasedSkeletonMatrix(cv::Mat skelMat, int limbid);
cv::Mat jointbasedSkeletonMatrix(cv::Mat skelMat, int limbid);