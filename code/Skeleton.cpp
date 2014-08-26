#include "Skeleton.h"
#include "CylinderBody.h"
#include "cvutil.h"

void Skeleton::calculateOffsetPoints(const CylinderBody& cylinderBody){
	for(int i=0;i<NUMLIMBS;++i){
		int f = getLimbmap()[i].first;
		int s = getLimbmap()[i].second;

		cv::Vec3f _a = mat_to_vec3(points.col(f));
		cv::Vec3f _b = mat_to_vec3(points.col(s));

		cv::Vec3f a = _b + cylinderBody.newLeftOffset_cyl[i] *	(_a - _b);
		cv::Vec3f b = _a + cylinderBody.newRightOffset_cyl[i] * (_b - _a);

		offsetPoints.ptr<float>(0)[i*2+0] = a(0);
		offsetPoints.ptr<float>(1)[i*2+0] = a(1);
		offsetPoints.ptr<float>(2)[i*2+0] = a(2);
		offsetPoints.ptr<float>(3)[i*2+0] = 1;

		offsetPoints.ptr<float>(0)[i*2+1] = b(0);
		offsetPoints.ptr<float>(1)[i*2+1] = b(1);
		offsetPoints.ptr<float>(2)[i*2+1] = b(2);
		offsetPoints.ptr<float>(3)[i*2+1] = 1;
	}

	offsetPointsCalculated = true;
};