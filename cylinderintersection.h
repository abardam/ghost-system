#pragma once
#include <opencv2\opencv.hpp>

#define TEXTURE_SEARCH_DEPTH 25

//2d point to 3d ray, given a 3x3 inverse camera matrix
cv::Vec3f raycast(cv::Vec2f pt, cv::Mat invCameraMatrix);

//ray from the origin to plane defined by pt and vec
int rayPlaneIntersection(cv::Vec3f ray, cv::Vec3f plane_Pt, cv::Vec3f plane_Vec, cv::Vec3f * out);

int rayCylinder(cv::Vec3f ray, cv::Vec3f cyl_a, cv::Vec3f cyl_b, float radius, cv::Vec3f * out);

//same as raycylinder but u have to pre-do the transformations
int rayCylinder2(cv::Mat transformation, cv::Vec3f ray, float radius, float height, cv::Vec3f * out);
