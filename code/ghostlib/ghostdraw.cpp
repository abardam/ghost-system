#include "ghostdraw.h"
#include "definitions.h"
#include "cvutil.h"
#include "cylinderprojection.h"
#include "ghostmapping.h"
#include "cylinderintersection.h"
#include "drawing.h"
#include "bodybuild.h"
#include "ghostcam.h"
#include "texturesearch.h"
#include "Limbrary.h"
#include "CylinderBody.h"

//parallelization
//#include <ppl.h>

void ghostdraw_prep(int frame, const cv::Mat& transform, int texSearchDepth, int wtType, const std::vector<SkeleVideoFrame>& vidRecord, const std::vector<Skeleton>& wcSkeletons, const CylinderBody& cylinderBody, const Limbrary& limbrary, cv::Vec3f a[NUMLIMBS], cv::Vec3f b[NUMLIMBS], float facing[NUMLIMBS], ScoreList scoreList[NUMLIMBS], cv::Point offsets[NUMLIMBS], cv::Mat fromPixels[NUMLIMBS], std::vector<cv::Vec3s>& fromPixels_2d_v, int limits[NUMLIMBS]){

	if(!wcSkeletons[frame].offsetPointsCalculated){
		std::cerr << "error! offset points not calculated\n";
		throw;
	}

	cv::Mat transformedOffsetPoints = transform * wcSkeletons[frame].offsetPoints;

	for(int i=0;i<NUMLIMBS;++i){

		//int f = getLimbmap()[i].first;
		//int s = getLimbmap()[i].second;
		//
		//cv::Mat _a = wcSkeletons[frame].points.col(f);
		//cv::Mat _b = wcSkeletons[frame].points.col(s);
		//
		//cv::Mat __a = _b + cylinderBody.newLeftOffset_cyl[i] * (_a - _b);
		//cv::Mat __b = _a + cylinderBody.newRightOffset_cyl[i] * (_b - _a);
		//
		//a[i] = mat_to_vec3(transform * __a);
		//b[i] = mat_to_vec3(transform * __b);

		a[i] = mat_to_vec3(transformedOffsetPoints.col(i*2+0));
		b[i] = mat_to_vec3(transformedOffsetPoints.col(i*2+1));

		float radius = cylinderBody.newPartRadii_cyl[i];

		CroppedCvMat source = limbrary.getFrameLimbs(frame)[i];
		Skeleton skele = vidRecord[frame].kinectPoints;
		skele.points = transform * skele.points;
			
		facing[i] = tempCalcFacing(i, skele); //13 us
		offsets[i] = source.offset;

			//fromPixels_v.clear(); //not using it
			std::vector<cv::Vec3f> fromPixels_v;
			fromPixels_v.reserve(2048);
			scoreList[i] = sortFrames(skele, vidRecord, limbrary, i, texSearchDepth, true, wtType); //105.311 us
			//fromPixels[i] = cylinder_to_pts(a[i],b[i],radius,source.offset,&fromPixels_v,&(fromPixels_2d_v)); //6000 us
			PixelPolygon p;
			fromPixels[i] = cylinder_to_pts(a[i],b[i],radius,source.offset,&p,&fromPixels_v,&(fromPixels_2d_v)); 
			limits[i] = fromPixels_2d_v.size();
	}
}

void ghostdraw_parallel(int frame, cv::Mat transform, std::vector<SkeleVideoFrame>& vidRecord, std::vector<Skeleton>& wcSkeletons, CylinderBody& cylinderBody, Limbrary& limbrary, cv::Mat& draw, cv::Mat& zBuf, unsigned char options, ScoreList * scoreListOut){

	
	int wtType = GH_WT_JOINT;

	if(options & GD_NOWEIGHT){
		wtType = GH_WT_NONE;
	}

	int texSearchDepth;
	int blendType;

	if(options & GD_NOLIMBRARY)
	{
		texSearchDepth = 1;
		blendType = CMPC_NO_OCCLUSION;
	}else if(options & GD_NOBLEND)
	{
		texSearchDepth = TEXTURE_SEARCH_DEPTH_SMALL;
		blendType = CMPC_BLEND_NONE;
	}
	else{
		texSearchDepth = TEXTURE_SEARCH_DEPTH_SMALL;
		blendType = CMPC_BLEND_1;
	}

	//cv::Mat zBuf(480, 640, CV_16U, cv::Scalar(MAXDEPTH)); moved inside func

	size_t n = NUMLIMBS;
	PixelColorMap from_color[NUMLIMBS];
	ScoreList scoreList[NUMLIMBS];

	cv::Vec3f a[NUMLIMBS];
	cv::Vec3f b[NUMLIMBS];
	float facing[NUMLIMBS];
	cv::Point offsets[NUMLIMBS];

	cv::Mat fromPixels[NUMLIMBS];
	//std::vector<cv::Vec3s> fromPixels_2d_v[NUMLIMBS];
	std::vector<cv::Vec3s> fromPixels_2d_v;
	fromPixels_2d_v.reserve(1048576);
	//std::vector<cv::Vec3f> fromPixels_v;

	int limits[NUMLIMBS];

	if(options & GD_DRAW)
	{
		ghostdraw_prep(frame, transform, texSearchDepth, wtType, vidRecord, wcSkeletons, cylinderBody, limbrary, a, b, facing, scoreList, offsets, fromPixels, fromPixels_2d_v, limits);
		
		cylinderMapPixelsColor_parallel_orig(a, b, facing, scoreList, offsets, 
			&vidRecord, &cylinderBody, &limbrary, blendType, fromPixels, fromPixels_2d_v, limits, draw, zBuf /*from_color*/);
	}


	if(options & GD_CYL){

		if(!wcSkeletons[frame].offsetPointsCalculated){
			std::cerr << "error! offset points not calculated\n";
			throw;
		}

		cv::Mat transformedOffsetPoints = transform * wcSkeletons[frame].offsetPoints;

		for(int i=0;i<NUMLIMBS;++i){
			

			//int f = getLimbmap()[i].first;
			//int s = getLimbmap()[i].second;
			//
			//cv::Mat _a = (wcSkeletons[frame].points.col(f));
			//cv::Mat _b = (wcSkeletons[frame].points.col(s));
			//
			//cv::Mat __a = _b + cylinderBody.newLeftOffset_cyl[i] * (_a - _b);
			//cv::Mat __b = _a + cylinderBody.newRightOffset_cyl[i] * (_b - _a);
			//
			//cv::Vec3f a = mat_to_vec3(transform * (__a));
			//cv::Vec3f b = mat_to_vec3(transform * (__b));

			cv::Vec3f a = mat_to_vec3(transformedOffsetPoints.col(i*2+0));
			cv::Vec3f b = mat_to_vec3(transformedOffsetPoints.col(i*2+1));

			float radius = cylinderBody.newPartRadii_cyl[i];

			std::vector<Segment3f> segments = cylinder_to_segments(a, b, radius, 16);

			for(auto it=segments.begin(); it!=segments.end(); ++it){
			
				cv::line(draw, cv::Point(toScreen(it->first)), cv::Point(toScreen(it->second)), cv::Scalar(0,0,255));
			}
		}
	}

	if(scoreListOut != 0){
		for(int i=0;i<NUMLIMBS;++i){
			scoreListOut[i] = scoreList[i];
		}
	}
}


void ghostdraw(int frame, cv::Mat transform, std::vector<SkeleVideoFrame>& vidRecord, std::vector<Skeleton>& wcSkeletons, CylinderBody& cylinderBody, Limbrary& limbrary, cv::Mat draw, unsigned char options){
	
	int wtType = GH_WT_JOINT;

	if(options & GD_NOWEIGHT){
		wtType = GH_WT_NONE;
	}

	int texSearchDepth;
	int blendType;

	if(options & GD_NOLIMBRARY)
	{
		texSearchDepth = 1;
		blendType = CMPC_NO_OCCLUSION;
	}else if(options & GD_NOBLEND)
	{
		texSearchDepth = TEXTURE_SEARCH_DEPTH;
		blendType = CMPC_BLEND_NONE;
	}
	else{
		texSearchDepth = TEXTURE_SEARCH_DEPTH;
		blendType = CMPC_BLEND_1;
	}

	cv::Mat zBuf(480, 640, CV_16U, cv::Scalar(MAXDEPTH));

	size_t n = NUMLIMBS;
	PixelColorMap from_color[NUMLIMBS];

	//concurrency::parallel_for(size_t(0), n, [&](size_t i){
	for(int i=0;i<NUMLIMBS;++i){

		int f = getLimbmap()[i].first;
		int s = getLimbmap()[i].second;

		cv::Vec3f _a = mat_to_vec3(wcSkeletons[frame].points.col(f));
		cv::Vec3f _b = mat_to_vec3(wcSkeletons[frame].points.col(s));

		cv::Vec3f __a = _b + cylinderBody.newLeftOffset_cyl[i] * (_a - _b);
		cv::Vec3f __b = _a + cylinderBody.newRightOffset_cyl[i] * (_b - _a);

		cv::Vec3f a = mat_to_vec3(transform * vec3_to_mat4(__a));
		cv::Vec3f b = mat_to_vec3(transform * vec3_to_mat4(__b));

		float radius = cylinderBody.newPartRadii_cyl[i];

		if(options & GD_DRAW)
		{
			CroppedCvMat source = limbrary.getFrameLimbs(frame)[i];
			Skeleton skele = vidRecord[frame].kinectPoints;
			skele.points = transform * skele.points;
			
			float facing = tempCalcFacing(i, skele);

			ScoreList scoreList = sortFrames(skele, vidRecord, limbrary, i, texSearchDepth, true, wtType);
			from_color[i] = cylinderMapPixelsColor(a, b, radius, i, facing, scoreList, source.offset, 
			&vidRecord, &cylinderBody, &limbrary, blendType);
			//parallelized
			//PixelColorMap from_color;
			//ScoreList scoreList = sortFrames(skele, &vidRecord, &limbrary, i, texSearchDepth, true, wtType);
			//from_color = cylinderMapPixelsColor(a, b, radius, i, facing, scoreList, source.offset, 
			//	&vidRecord, &cylinderBody, &limbrary, blendType);
			//
			//for(int j=0;j<from_color.first.size();++j){
			//
			//	cv::Point pt(from_color.first[j](0), from_color.first[j](1));
			//	pt += source.offset;
			//	unsigned short& ptDepth = zBuf.ptr<unsigned short>(pt.y)[pt.x];
			//
			//	unsigned short new_depth = from_color.first[j](2);
			//	unsigned short old_depth = ptDepth;
			//
			//	if(new_depth < old_depth && new_depth > 0){
			//
			//		//cvDrawPoint(draw, pt, from_color.second[j]); //changed to UNSAFE version (below)
			//		cv::Vec3b& ptColor = draw.ptr<cv::Vec3b>(pt.y)[pt.x];
			//		for(int k=0;k<3;++k){
			//			ptColor(k) = from_color.second[j](k);
			//		}
			//		
			//		ptDepth = new_depth;
			//	
			//		//cv::imshow("pic", draw);
			//		//cv::waitKey(10);
			//	}
			//}

		}

		
		CroppedCvMat source = limbrary.getFrameLimbs(frame)[i];
		for(int j=0;j<from_color[i].first.size();++j){
	
			cv::Point pt(from_color[i].first[j](0), from_color[i].first[j](1));
			pt += source.offset;
			unsigned short& ptDepth = zBuf.ptr<unsigned short>(pt.y)[pt.x];
	
			if(from_color[i].first[j](2) < ptDepth && from_color[i].first[j](2) > 0){

#pragma omp critical
				{
					if(from_color[i].first[j](2) < ptDepth && from_color[i].first[j](2) > 0){
						//cvDrawPoint(draw, pt, from_color.second[j]); //changed to UNSAFE version (below)
						cv::Vec3b& ptColor = draw.ptr<cv::Vec3b>(pt.y)[pt.x];
						for(int k=0;k<3;++k){
							ptColor(k) = from_color[i].second[j](k);
						}
			
						ptDepth = from_color[i].first[j](2);
					}
				}
				//cv::imshow("pic", draw);
				//cv::waitKey(10);
			}
		}
	}

	if(options & GD_CYL){

		for(int i=0;i<NUMLIMBS;++i){
			

			int f = getLimbmap()[i].first;
			int s = getLimbmap()[i].second;

			cv::Vec3f _a = mat_to_vec3(wcSkeletons[frame].points.col(f));
			cv::Vec3f _b = mat_to_vec3(wcSkeletons[frame].points.col(s));

			cv::Vec3f __a = _b + cylinderBody.newLeftOffset_cyl[i] * (_a - _b);
			cv::Vec3f __b = _a + cylinderBody.newRightOffset_cyl[i] * (_b - _a);

			cv::Vec3f a = mat_to_vec3(transform * vec3_to_mat4(__a));
			cv::Vec3f b = mat_to_vec3(transform * vec3_to_mat4(__b));

			float radius = cylinderBody.newPartRadii_cyl[i];

			std::vector<Segment3f> segments = cylinder_to_segments(a, b, radius, 16);

			for(auto it=segments.begin(); it!=segments.end(); ++it){
			
				cv::line(draw, cv::Point(toScreen(it->first)), cv::Point(toScreen(it->second)), cv::Scalar(255,255,0));
			}
		}
	}
}