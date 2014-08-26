#pragma once
#include <vector>
#include <opencv2\opencv.hpp>

#include "definitions.h"
#include "Skeleton.h"

#define WINDOW 5
#define EXP_A 0.1

std::vector<double> exp_smooth(const std::vector<double>& x);
cv::Mat exp_smooth(const cv::Mat x, double exp_a);

//should change this to work on wcSkeletons
void smoothKinectPoints(std::vector<Skeleton>& wcSkeletons, double exp_a  = EXP_A);