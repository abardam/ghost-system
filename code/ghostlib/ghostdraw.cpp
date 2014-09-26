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

void ghostdraw_prep(int frame, const cv::Mat& transform, int texSearchDepth, int wtType, const std::vector<SkeleVideoFrame>& vidRecord, const std::vector<Skeleton>& wcSkeletons, const CylinderBody& cylinderBody, Limbrary& limbrary, cv::Vec3f a_arr[NUMLIMBS], cv::Vec3f b_arr[NUMLIMBS], float facing[NUMLIMBS], ScoreList scoreList[NUMLIMBS], cv::Point offsets[NUMLIMBS], cv::Mat fromPixels[NUMLIMBS], std::vector<cv::Vec3s>& fromPixels_2d_v, int limits[NUMLIMBS], std::vector<cv::Rect>* boundingRects){

	if(!wcSkeletons[frame].offsetPointsCalculated){
		std::cerr << "error! offset points not calculated\n";
		throw;
	}

	cv::Mat transformedOffsetPoints = transform * wcSkeletons[frame].offsetPoints;
	Skeleton skele = wcSkeletons[frame];
	skele.points = transform * skele.points;

	
	//debug; delete later
	//cv::Mat b = normalizeSkeleton(skele.points);
	//cv::Mat searchShow(480, 640, CV_8UC3, cv::Scalar(255,255,255));
	//for(int joint=0;joint<NUMJOINTS;++joint){
	//	b.ptr<float>(2)[joint] += 4;
	//	cv::Scalar color = joint==0?cv::Scalar(0,244,0):cv::Scalar(255,0,0);
	//	cv::line(searchShow, cv::Point(toScreen(b.col(joint))), cv::Point(toScreen(cv::Vec3f(0,0,4))), color);
	//}
	//cv::imshow("search show", searchShow);
	//static int a = 0;
	//std::stringstream ss;
	//ss << "texsearch/texsearch" << a << ".png";
	//cv::imwrite(ss.str(), searchShow);
	//++a;

	for(int limb=0;limb<NUMLIMBS;++limb){

		int lastLimbLimit = (limb>0?limits[limb-1]:0);
		limits[limb] = lastLimbLimit;

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

		a_arr[limb] = mat_to_vec3(transformedOffsetPoints.col(limb*2+0));
		b_arr[limb] = mat_to_vec3(transformedOffsetPoints.col(limb*2+1));


		float radius = cylinderBody.newPartRadii_cyl[limb] * cylinderBody.radiusModifier;

		CroppedCvMat source = limbrary.frames[frame][limb];
		//Skeleton skele = vidRecord[frame].kinectPoints;
			
		facing [limb] = tempCalcFacing(limb, skele); //13 us
		source.offset = cv::Point(0,0);
		offsets[limb] = source.offset;

			//fromPixels_v.clear(); //not using it
			//std::vector<cv::Vec3f> fromPixels_v;
			//fromPixels_v.reserve(2048);

			scoreList[limb] = sortFrames(skele, vidRecord, limbrary, limb, texSearchDepth, true, wtType); 
			


			//fromPixels[i] = cylinder_to_pts(a[i],b[i],radius,source.offset,&fromPixels_v,&(fromPixels_2d_v)); //6000 us
			cv::Rect p;

			{
				//fromPixels[i] = cylinder_to_pts(a[i],b[i],radius,source.offset,&p,&fromPixels_v,&(fromPixels_2d_v)); 
				//cv::Mat ret(4, 0, CV_32F, cv::Scalar(1));

				cv::Vec3f a,b;

				if(cv::norm(a_arr[limb]) < cv::norm(b_arr[limb])){
					a = a_arr[limb];
					b = b_arr[limb];
				}else{
					a = b_arr[limb];
					b = a_arr[limb];
				}

				cv::Vec3f a_b = cv::normalize(b-a);

				//std::vector<cv::Vec3f> pts = cylinder_to_vertices(a, b, radius,8);
				//std::vector<cv::Vec2f> pts2 = vec3f_to_2f(pts, cv::Vec2f(voff.x, voff.y));
				//*p = polygon_contains_pixels(pts2);


				std::vector<Segment3f> pts = cylinder_to_segments(a-radius*a_b, b+radius*a_b, radius,8);
				std::vector<Segment2f> pts2 = segment3f_to_2f(pts, cv::Vec2f(source.offset.x, source.offset.y), getCameraMatrixScene());
				if(pts2.empty()){
					fromPixels[limb] = cv::Mat(4, 0, CV_32F, cv::Scalar(1));;
					continue;
				}
				cv::Rect r = cv::boundingRect(segments_to_points(pts2));
				if (boundingRects != NULL){
					boundingRects->push_back(r);
				}

				//transform the space:
				cv::Vec3f cyl_axis = b - a;

				float height = cv::norm(cyl_axis);
				cv::Mat transformation_inv;
				cv::Mat transformation = segmentZeroTransformation(a,b,&transformation_inv);


				//cv::Vec3f origin_trans = mat_to_vec(transformation.col(3)); //mat_to_vec3(transformation * vec3_to_mat4(cv::Vec3f(0,0,0)));
				float origin_trans[3];
				for(int i=0;i<3;++i){
					origin_trans[i] = *(transformation.ptr<float>(i)+3);
				}

				float C = origin_trans[0]*origin_trans[0] + origin_trans[1]*origin_trans[1] - radius*radius;

				int limbpicWidth  = r.width;
				int limbpicHeight = r.height;
	
				//std::cout << p->x_offset << " " << limbpicWidth << " " << p->lo_y << " " << limbpicHeight << std::endl;

				if (limbpicWidth <= 0 || limbpicHeight <= 0 || limbpicWidth * limbpicHeight * 4 <= 0 ) continue;

				cv::Mat rayMat;
				try{
					rayMat = cv::Mat (4, limbpicHeight*limbpicWidth, CV_32F);
				}
				catch (std::exception){
					continue;
				}

				for (int _x = 0; _x < limbpicWidth; ++_x){
					for (int _y = 0; _y < limbpicHeight; ++_y){
						int y = _y + r.y;
						int x = _x + r.x;

						rayMat.ptr<float>(0)[_y*limbpicWidth + _x] = x + source.offset.x;
						rayMat.ptr<float>(1)[_y*limbpicWidth + _x] = y + source.offset.y;
						rayMat.ptr<float>(2)[_y*limbpicWidth + _x] = 1;
						rayMat.ptr<float>(3)[_y*limbpicWidth + _x] = 1;
					}
				}

				rayMat = transformation * getInvCameraMatrixScene() * rayMat;


				std::vector<float> fromPixels_f(limbpicHeight*limbpicWidth*4);
				int valid_count = 0;

				for(int _x = 0; _x < limbpicWidth; ++_x){
					for(int _y = 0; _y < limbpicHeight; ++_y){
						int y = _y + r.y;
						int x = _x + r.x;

						//cv::Vec3f pt(x+voff.x,y+voff.y,1);
						//cv::Vec3f ray = mat_to_vec(invCameraMatrix * cv::Mat(pt)); replaced with lerp
						//cv::Vec3f ray = lerpPoint(x+voff.x,y+voff.y,boundingBox,lc);
						//cv::Mat ray_trans1 = transformation * vec3_to_mat4(ray);

						//cv::Mat ray_trans = rayMat.col(_y*limbpicWidth+_x).clone();
						int ind = _y*limbpicWidth+_x;
						float ray_trans_f[] = {rayMat.ptr<float>(0)[ind], rayMat.ptr<float>(1)[ind], rayMat.ptr<float>(2)[ind], rayMat.ptr<float>(3)[ind]};
						//cv::Vec3f ptProj; //it's inefficient to project the points individually; better to put em all in a big matrix before multiplying the inverse transformation
						float retf[4];

						//int res = rayCylinder2(transformation, ray, radius, height, &ptProj);

						int res;
						{
							//int res = rayCylinder3_c(origin_trans, /*ray_trans.ptr<float>()*/ray_trans_f, transformation_inv, C, height, &ptProj);

							bool res2 = rayCylinderClosestIntersectionPoint_c(origin_trans, ray_trans_f, C, height, retf);


							if(res2){
								retf[3] = 1;
								//ptProj = mat_to_vec3(transformation_inv*cv::Mat(4,1,CV_32F,retf));
								res = 1;
							}
							else res = 0;

						}

						if(res == 1){
							//see above note on ptProj
							//fromPixels_v.push_back(ptProj);
							////cv::Vec3s xyDepth(pt(0)-voff.x, pt(1)-voff.y, ptProj(2) * FLOAT_TO_DEPTH);
							//cv::Vec3s xyDepth(x, y, ptProj(2) * FLOAT_TO_DEPTH);
							cv::Vec3s xyDepth(x, y, 0);
							fromPixels_2d_v.push_back(xyDepth);

							fromPixels_f[valid_count*4+0] = retf[0];
							fromPixels_f[valid_count*4+1] = retf[1];
							fromPixels_f[valid_count*4+2] = retf[2];
							fromPixels_f[valid_count*4+3] = retf[3];

							++valid_count;
						}
					}
				}

				if (valid_count * 4 <= 0){
					continue;
				}
				cv::Mat ret;

				try{
					ret = cv::Mat(valid_count, 4, CV_32F, fromPixels_f.data()).t();
				}
				catch (std::exception){
					continue;
				}
				//for(int j=0;j<4;++j){
				//	float * retptr = ret.ptr<float>(j);
				//	for(int i=0;i<fromPixels_f.size();++i){
				//		*(retptr+i) = fromPixels_f[i][j];
				//	}
				//}

				ret = transformation_inv * ret;
				//cv::Mat m2DPoints = getCameraMatrixScene() * ret;
				//std::vector<cv::Vec3s> fromPixels_2d_v2;

				//cv::divide(ret.row(0), ret.row(3), ret.row(0)); //somehow the last row is already 1
				//cv::divide(ret.row(1), ret.row(3), ret.row(1));
				//cv::divide(ret.row(2), ret.row(3), ret.row(2));
				//cv::divide(ret.row(3), ret.row(3), ret.row(3));


				for(int j=0;j<ret.cols;++j){
					int ind = j+lastLimbLimit;
					fromPixels_2d_v[ind](2) = ret.ptr<float>(2)[j] * FLOAT_TO_DEPTH;

					//int x = m2DPoints.ptr<float>(0)[j] / m2DPoints.ptr<float>(2)[j];
					//int y = m2DPoints.ptr<float>(1)[j] / m2DPoints.ptr<float>(2)[j];
					//int depth = ret.ptr<float>(2)[j] * FLOAT_TO_DEPTH;
					//
					//cv::Vec3s xyDepth(x, y, depth); 
					////fromPixels_2d_v2.push_back(xyDepth);
					//fromPixels_2d_v[ind](0) = x;
					//fromPixels_2d_v[ind](1) = y;
				}

				//std::cout << fromPixels_2d_v[lastLimbLimit] - fromPixels_2d_v2[0] << std::endl;
				//ret.create(4,fromPixels_v.size(),CV_32F);
				//for(int j=0;j<4;++j){
				//	float * retptr = ret.ptr<float>(j);
				//	for(int i=0;i<fromPixels_v.size();++i){
				//		//ret.at<float>(j,i) = (*fromPixels)[i](j);
				//		if(j<3)
				//			*(retptr+i) = fromPixels_v[i](j);
				//		else
				//			*(retptr+i) = 1;
				//	}
				//}

				fromPixels[limb] = ret;
			}


			limits[limb] = fromPixels_2d_v.size();
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

	std::vector<cv::Rect> boundingRects;

	if(options & GD_DRAW)
	{
		ghostdraw_prep(frame, transform, texSearchDepth, wtType, vidRecord, wcSkeletons, cylinderBody, limbrary, a, b, facing, scoreList, offsets, fromPixels, fromPixels_2d_v, limits, &boundingRects);

		if (options&GD_NOCOLOR){
			for (auto it = fromPixels_2d_v.begin(); it != fromPixels_2d_v.end(); ++it){
				if (CLAMP_SIZE((*it)(0), (*it)(1), draw.cols, draw.rows)){
					float depthRatio = ((*it)(2) + 0.0) / MAXDEPTH;
					draw.ptr<cv::Vec4b>((*it)(1))[(*it)(0)] = cv::Vec4b(depthRatio * 255, 100 + depthRatio * 155, 200 + depthRatio * 55, 255);
				}
			}
		}
		else{

			cylinderMapPixelsColor_parallel_orig(a, b, facing, scoreList, offsets,
				&vidRecord, &cylinderBody, &limbrary, blendType, fromPixels, fromPixels_2d_v, limits, draw, zBuf /*from_color*/);
		}
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

			float radius = cylinderBody.newPartRadii_cyl[i] * cylinderBody.radiusModifier;

			std::vector<Segment3f> segments = cylinder_to_segments(a, b, radius, 16);

			for(auto it=segments.begin(); it!=segments.end(); ++it){
			
				cv::line(draw, cv::Point(mat4_to_vec2(getCameraMatrixScene()*vec3_to_mat4(it->first))), cv::Point(mat4_to_vec2(getCameraMatrixScene()*vec3_to_mat4(it->second))), cv::Scalar(0,0,255,255));
			}

			cv::rectangle(draw, boundingRects[i], cv::Scalar(0, 255, 255, 255), 1);
		}
	}

	if(scoreListOut != 0){
		for(int i=0;i<NUMLIMBS;++i){
			scoreListOut[i] = scoreList[i];
		}
	}
}

#if 0
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

#endif