#pragma once

#include <opencv2\opencv.hpp>
#include "texturesearch.h"

class parallel_pixelMapping:
	public cv::ParallelLoopBody
{
private:
	std::vector<SkeleVideoFrame> * vidRecord;
	CylinderBody * cylinderBody;
	Limbrary * limbrary;
	ScoreList * scoreList;

	cv::Vec3f * fromPixels;
	cv::Scalar * pixelColors;
	std::vector<bool> * erasevector;

	cv::Vec3f from_a, from_b;
	int limbid;
	float facing;
	float radius;

	int blendMode;
	int blendLimit;

public:
	parallel_pixelMapping(std::vector<SkeleVideoFrame> * vidRecord,
	CylinderBody * cylinderBody,
	Limbrary * limbrary,
	ScoreList * scoreList,

	cv::Vec3f * fromPixels,
	cv::Scalar * pixelColors,
	std::vector<bool> * erasevector,

	cv::Vec3f from_a, cv::Vec3f from_b,
	int limbid,
	float facing,
	float radius,

	int blendMode,
	int blendLimit);

	virtual void operator()(const cv::Range& r) const;
};