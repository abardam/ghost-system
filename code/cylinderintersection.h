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


//should be a copy of rayCylinder2 but the transformations are pre-done AND pre-applied
int rayCylinder3(float * origin_trans, float * ray_trans, cv::Mat transformation_inv, float radius, float height, cv::Vec3f * out);

//C = origin[0]^2 + origin[1]^2 - radius^2
int rayCylinder3_c(float * origin_trans, float * ray_trans, cv::Mat transformation_inv, float c, float height, cv::Vec3f * out);

bool rayCylinderClosestIntersectionPoint(float * origin, float * ray_, const float& radius, const float& height, float out[3]);

//C = origin[0]^2 + origin[1]^2 - radius^2
bool rayCylinderClosestIntersectionPoint_c(float * origin, float * ray_, float c, const float& height, float out[3]);