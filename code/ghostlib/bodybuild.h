#pragma once
#include "opencv2\opencv.hpp"
#include "definitions.h"
#include "CylinderBody.h"
#include "Skeleton.h"

#define IDEAL_MULT_TORSO 1.5
#define IDEAL_MULT_LIMB 0.5


//experimental. hardcoded matrix
cv::Mat expGetCameraMatrix();

struct BodyPartParam{
	float radius;
	float leftOffset;
	float rightOffset;
};

struct Cylinder{
	cv::Vec3f pt1;
	cv::Vec3f pt2;
	float leftOffset;
	float rightOffset;
	float radius;
	Cylinder(cv::Vec3f a, cv::Vec3f b, float r):pt1(a), pt2(b), radius(r){};
	Cylinder():pt1(0,0),pt2(0,0),radius(0){};
};

template <typename T, int S>
struct Segment{
	cv::Vec<T,S> first;
	cv::Vec<T,S> second;

	Segment(){}
	Segment(cv::Vec<T,S> f, cv::Vec<T,S> s):first(f),second(s){}

	bool operator==(Segment<T,S> s){
		return first == s.first && second == s.second;
	};
};

typedef Segment<float, 3> Segment3f;
typedef Segment<float, 2> Segment2f;

struct PixelPolygon{
	std::vector<int> hi;
	std::vector<int> lo;
	int x_offset;
	int hi_y;
	int lo_y;
	PixelPolygon():
		hi_y(-1),lo_y(-1){}
};

std::vector<BodyPartParam> rectFitting(Skeleton skeletonPositions, CroppedCvMat im, CylinderBody * cylinderBody);
std::vector<Cylinder> projectedCylinderFitting(Skeleton skeletonPositions, CroppedCvMat im, CylinderBody * cylinderBody);

//convenience function
std::vector<Segment2f> segment3f_to_2f(std::vector<Segment3f> pts, cv::Vec2f offset);
std::vector<cv::Vec2f> vec3f_to_2f(std::vector<cv::Vec3f> pts, cv::Vec2f offset, cv::Mat& cameraMatrix);

//returns a rectangle version of the cylinder. note: transform the points beforehand.
std::vector<cv::Vec3f> cylinder_to_rectangle(cv::Vec3f a, cv::Vec3f b, float radius);

std::vector<Segment3f> cylinder_to_segments(cv::Vec3f a, cv::Vec3f b, float radius, int numsegments=4);
std::vector<cv::Vec3f> cylinder_to_vertices(cv::Vec3f a, cv::Vec3f b, float radius, int numsegments=4);

//returns y given x, and the 2 segment endpoints
float point_on_segment(cv::Vec2f a, cv::Vec2f b, float x, bool * on);

//given a vector of vertices (of a polygon)
//over the width of the polygon, returns the highest and lowest Y-value
void polygon_contains_pixels(std::vector<cv::Vec2f> points, std::vector<float> * hi, std::vector<float> * lo, float * x_offset);
PixelPolygon polygon_contains_pixels(std::vector<Segment2f> polygon);

template <typename T, int S>
std::pair<Segment<T,S>,Segment<T,S>> subdivideSegment(Segment<T,S> s){
	cv::Vec<T,S> mid;
	for(int i=0;i<S;++i){
		mid[i] = (s.first[i] + s.second[i])/2;
	}
	Segment<T,S> a;
	Segment<T,S> b;
	a.first = s.first;
	a.second = mid;
	b.first = mid;
	b.second = s.second;

	return std::pair<Segment<T,S>,Segment<T,S>>(a,b);
};

template <typename T, int S>
std::vector<cv::Vec<T,S>> segments_to_points(std::vector<Segment<T,S>> polygon){
	std::vector<cv::Vec<T,S>> ret;

	for(auto it=polygon.begin(); it!=polygon.end(); ++it){
		bool pushfirst = true, pushsecond = true;
		for(auto it2=ret.begin(); it2 != ret.end(); ++it2){
			if((*it2) == it->first)
				pushfirst = false;
			if(*it2 == it->second)
				pushsecond = false;
		}
		if(pushfirst)
			ret.push_back(it->first);
		if(pushsecond)
			ret.push_back(it->second);
	}

	return ret;
};

//from loader.cpp
bool buildDepth(Skeleton skeletonPositions, cv::Mat depthIm, CroppedCvMat colorIm, CylinderBody * cb);

//pixelpolygon methods