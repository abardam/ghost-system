#pragma once
#include <opencv2\opencv.hpp>

void setCameraMatrixScene(float param0, float param1, float param2, float param3, float width, float height);
void setCameraMatrixScene(cv::Mat);
void setCameraMatrixTexture(cv::Mat);
//void calculateCameraMatrix();

cv::Mat& getCameraMatrixScene();
cv::Mat& getInvCameraMatrixScene();
cv::Mat& getCameraMatrixTexture();
cv::Mat& getInvCameraMatrixTexture();

//cv::Vec2f toScreen(cv::Vec3f v);