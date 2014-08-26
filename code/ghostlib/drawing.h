#pragma once
#include "opencv2\opencv.hpp"

//given 2xN matrix of points, convert to outline
std::vector<cv::Point> cloudOutline(cv::Mat _2D, int width, int height);

//i draw a lot of points
void cvDrawPoint(cv::Mat& im, cv::Point p, cv::Scalar color);