#include <opencv2\opencv.hpp>
#include "definitions.h"
#include "CylinderBody.h"

void cylinderPoints(int frame, int limbid, cv::Mat transform, cv::Mat * pts, const std::vector<Skeleton>& wcSkeletons, const CylinderBody& cylinderBody);
void renderLimb(int frame, int limbid, cv::Mat transform, const std::vector<Skeleton>& wcSkeletons, const CylinderBody& cylinderBody);

cv::Mat cylinder_to_ptsGL(cv::Vec3f a, cv::Vec3f b, float radius, cv::Point voff, std::vector<cv::Vec3f> * fromPixels, std::vector<cv::Vec3s> * fromPixels_2d_v);
void initGL(int argc, char ** argv);