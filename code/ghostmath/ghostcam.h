#pragma once
#include <opencv2\opencv.hpp>

void setCameraMatrix(float param0, float param1, float param2, float param3, float width, float height);
void setCameraMatrix(cv::Mat);
//void calculateCameraMatrix();

cv::Mat& getCameraMatrix();
cv::Mat& getInvCameraMatrix();

cv::Vec2f toScreen(cv::Vec3f v);