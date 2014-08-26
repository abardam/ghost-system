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
#include "camlerp.h"


//parallelization
//#include <ppl.h>

void ghostdraw_prep(int frame, const cv::Mat& transform, int texSearchDepth, int wtType, const std::vector<SkeleVideoFrame>& vidRecord, const std::vector<Skeleton>& wcSkeletons, const CylinderBody& cylinderBody, const Limbrary& limbrary, cv::Vec3f a_arr[NUMLIMBS], cv::Vec3f b_arr[NUMLIMBS], float facing[NUMLIMBS], ScoreList scoreList[NUMLIMBS], cv::Point offsets[NUMLIMBS], cv::Mat fromPixels[NUMLIMBS], std::vector<cv::Vec3s>& fromPixels_2d_v, int limits[NUMLIMBS]){

	if(!wcSkeletons[frame].offsetPointsCalculated){
		std::cerr << "error! offset points not calculated\n";
		throw;
	}

	cv::Mat transformedOffsetPoints = transform * wcSkeletons[frame].offsetPoints;


	ctimer cintersect, cclone, ccylinder, cres;

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

		a_arr[i] = mat_to_vec3(transformedOffsetPoints.col(i*2+0));
		b_arr[i] = mat_to_vec3(transformedOffsetPoints.col(i*2+1));

		float radius = cylinderBody.newPartRadii_cyl[i];

		CroppedCvMat source = limbrary.frames[frame][i];
		Skeleton skele = vidRecord[frame].kinectPoints;
		skele.points = transform * skele.points;
			
		facing[i] = tempCalcFacing(i, skele); //13 us
		offsets[i] = source.offset;

			//fromPixels_v.clear(); //not using it
			std::vector<cv::Vec3f> fromPixels_v;
			fromPixels_v.reserve(2048);

			scoreList[i] = sortFrames(skele, vidRecord, limbrary, i, texSearchDepth, true, wtType); //105.311 us

			//fromPixels[i] = cylinder_to_pts(a[i],b[i],radius,source.offset,&fromPixels_v,&(fromPixels_2d_v)); //6000 us
			cv::Rect p;

			{
				//fromPixels[i] = cylinder_to_pts(a[i],b[i],radius,source.offset,&p,&fromPixels_v,&(fromPixels_2d_v)); 
				cv::Mat ret(4, 0, CV_32F, cv::Scalar(1));
				cv::Mat invCameraMatrix = (getInvCameraMatrix()); 

				cv::Vec3f a,b;

				if(a_arr[i](2) < b_arr[i](2)){
					a = a_arr[i];
					b = b_arr[i];
				}else{
					a = b_arr[i];
					b = a_arr[i];
				}

				//std::vector<cv::Vec3f> pts = cylinder_to_vertices(a, b, radius,8);
				//std::vector<cv::Vec2f> pts2 = vec3f_to_2f(pts, cv::Vec2f(voff.x, voff.y));
				//*p = polygon_contains_pixels(pts2);


				std::vector<Segment3f> pts = cylinder_to_segments(a, b, radius,8);
				std::vector<Segment2f> pts2 = segment3f_to_2f(pts, cv::Vec2f(source.offset.x, source.offset.y));
				if(pts2.empty()){
					fromPixels[i] = ret;
					break;
				}
				cv::Rect r = cv::boundingRect(segments_to_points(pts2));


				//transform the space:
				cv::Vec3f cyl_axis = b - a;

				float height = cv::norm(cyl_axis);
				cv::Mat transformation = segmentZeroTransformation(a,b);
				cv::Mat transformation_inv = transformation.inv();


				//cv::Vec3f origin_trans = mat_to_vec(transformation.col(3)); //mat_to_vec3(transformation * vec3_to_mat4(cv::Vec3f(0,0,0)));
				float origin_trans[3];
				for(int i=0;i<3;++i){
					origin_trans[i] = *(transformation.ptr<float>(i)+3);
				}

				float C = origin_trans[0]*origin_trans[0] + origin_trans[1]*origin_trans[1] - radius*radius;

				int limbpicWidth  = r.width;
				int limbpicHeight = r.height;
	
				//std::cout << p->x_offset << " " << limbpicWidth << " " << p->lo_y << " " << limbpicHeight << std::endl;

				cv::Rect boundingBox(0,0,WIDTH,HEIGHT);
				LerpCorners lc = generateLerpCorners(boundingBox);
	
				cv::Mat rayMat(4, limbpicHeight*limbpicWidth, CV_32F);

				for(int _x = 0; _x < limbpicWidth; ++_x){
					for(int _y = 0; _y < limbpicHeight; ++_y){
						int y = _y + r.y;
						int x = _x + r.x;
						cv::Vec3f ray = lerpPoint(x+source.offset.x,y+source.offset.y,boundingBox,lc);

						rayMat.ptr<float>(0)[_y*limbpicWidth+_x] = ray(0);
						rayMat.ptr<float>(1)[_y*limbpicWidth+_x] = ray(1);
						rayMat.ptr<float>(2)[_y*limbpicWidth+_x] = 1;
						rayMat.ptr<float>(3)[_y*limbpicWidth+_x] = 1;

					}
				}

				rayMat = transformation * rayMat;

				cintersect.start();
				for(int _x = 0; _x < limbpicWidth; ++_x){
					for(int _y = 0; _y < limbpicHeight; ++_y){
						int y = _y + r.y;
						int x = _x + r.x;

						//cv::Vec3f pt(x+voff.x,y+voff.y,1);
						//cv::Vec3f ray = mat_to_vec(invCameraMatrix * cv::Mat(pt)); replaced with lerp
						//cv::Vec3f ray = lerpPoint(x+voff.x,y+voff.y,boundingBox,lc);
						//cv::Mat ray_trans1 = transformation * vec3_to_mat4(ray);

						cclone.start();
						//cv::Mat ray_trans = rayMat.col(_y*limbpicWidth+_x).clone();
						int ind = _y*limbpicWidth+_x;
						float ray_trans_f[] = {rayMat.ptr<float>(0)[ind], rayMat.ptr<float>(1)[ind], rayMat.ptr<float>(2)[ind], rayMat.ptr<float>(3)[ind]};
						cclone.end();
						cv::Vec3f ptProj;

						//int res = rayCylinder2(transformation, ray, radius, height, &ptProj);

						int res;
						{
							ccylinder.start();
							//int res = rayCylinder3_c(origin_trans, /*ray_trans.ptr<float>()*/ray_trans_f, transformation_inv, C, height, &ptProj);

							float retf[4];
							bool res2 = rayCylinderClosestIntersectionPoint_c(origin_trans, ray_trans_f, C, height, retf);
							ccylinder.end();


							cres.start();
							if(res2){
								retf[3] = 1;
								ptProj = mat_to_vec3(transformation_inv*cv::Mat(4,1,CV_32F,retf));
								res = 1;
							}
							else res = 0;
							cres.end();

						}

						if(res == 1){
							fromPixels_v.push_back(ptProj);
							//cv::Vec3s xyDepth(pt(0)-voff.x, pt(1)-voff.y, ptProj(2) * FLOAT_TO_DEPTH);
							cv::Vec3s xyDepth(x, y, ptProj(2) * FLOAT_TO_DEPTH);
							fromPixels_2d_v.push_back(xyDepth);
						}
					}
				}
				cintersect.end();

				ret.create(4,fromPixels_v.size(),CV_32F);
				for(int j=0;j<4;++j){
					float * retptr = ret.ptr<float>(j);
					for(int i=0;i<fromPixels_v.size();++i){
						//ret.at<float>(j,i) = (*fromPixels)[i](j);
						if(j<3)
							*(retptr+i) = fromPixels_v[i](j);
						else
							*(retptr+i) = 1;
					}
				}

				fromPixels[i] = ret;
			}


			limits[i] = fromPixels_2d_v.size();
	}
			std::cout << "  clone: " << cclone.c << "\n  cylinder: " << ccylinder.c << "\n  res: " << cres.c <<  "\n  intersect: " << cintersect.c << std::endl;

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
		clock_t t1 = clock();
		ghostdraw_prep(frame, transform, texSearchDepth, wtType, vidRecord, wcSkeletons, cylinderBody, limbrary, a, b, facing, scoreList, offsets, fromPixels, fromPixels_2d_v, limits);
		clock_t t2 = clock();
		std::cout << "ghostdraw_prep: " <<  t2-t1 << std::endl;

		t1 = clock();
		cylinderMapPixelsColor_parallel_orig(a, b, facing, scoreList, offsets, 
			&vidRecord, &cylinderBody, &limbrary, blendType, fromPixels, fromPixels_2d_v, limits, draw, zBuf /*from_color*/);
		t2 = clock();
		std::cout << "cylinderMapPixelsColor_parallel_orig: " <<  t2-t1 << std::endl;
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
			CroppedCvMat source = limbrary.frames[frame][i];
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

		
		CroppedCvMat source = limbrary.frames[frame][i];
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