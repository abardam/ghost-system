#include "cylinderprojection.h"
#include "cylinderintersection.h"
#include "cvutil.h"
#include "bodybuild.h"
#include "ghostcam.h"
#include "texturesearch.h"
#include "Limbrary.h"

//remove later
//used for facingHelper
#include "KinectManager.h"

cv::Mat cylinder_to_pts(cv::Vec3f a, cv::Vec3f b, float radius, cv::Point voff, PixelPolygon * p, std::vector<cv::Vec3f> * fromPixels, std::vector<cv::Vec3s> * fromPixels_2d_v){

	cv::Mat invCameraMatrix = (getInvCameraMatrix()); //todo: pre-invert

	std::vector<Segment3f> pts = cylinder_to_segments(a, b, radius,16);
	std::vector<Segment2f> pts2 = segment3f_to_2f(pts, cv::Vec2f(voff.x, voff.y));
	*p = polygon_contains_pixels(pts2);
			
	//transform the space:
	cv::Vec3f cyl_axis = b - a;

	float height = cv::norm(cyl_axis);
	cv::Mat transformation = segmentZeroTransformation(a,b);

	int limbpicWidth = p->hi.size();

	fromPixels->clear();
	fromPixels_2d_v->clear();

	for(int _x = 0; _x < limbpicWidth; ++_x){
		for(int y = p->lo[_x]; y < p->hi[_x]; ++y){
			int x = _x + p->x_offset;

			cv::Vec3f pt(x+voff.x,y+voff.y,1);
			cv::Vec3f ray = mat_to_vec(invCameraMatrix * cv::Mat(pt));
			cv::Vec3f ptProj;

			int res = rayCylinder2(transformation, ray, radius, height, &ptProj);
			if(res == 1){
				fromPixels->push_back(ptProj);
				cv::Vec3s xyDepth(pt(0)-voff.x, pt(1)-voff.y, ptProj(2) * FLOAT_TO_DEPTH);
				fromPixels_2d_v->push_back(xyDepth);
			}
		}
	}

	cv::Mat ret(4, fromPixels->size(), CV_32F, cv::Scalar(1));
	for(int i=0;i<fromPixels->size();++i){
		for(int j=0;j<3;++j){
			ret.at<float>(j,i) = (*fromPixels)[i](j);
		}
	}

	return ret;
}

cv::Mat pts_to_zBuffer(cv::Mat cylPts, cv::Point voff, cv::Point offset, unsigned int width, unsigned int height){
	cv::Mat cameraMatrix = getCameraMatrix();

	cv::Mat zBufferLocal(height, width, CV_16U, cv::Scalar(MAXDEPTH));

	for(int i=0;i<cylPts.cols;++i){
		cv::Vec3f ptProj = mat_to_vec(cylPts.col(i));

		cv::Vec2f ptProj2d = mat4_to_vec2(cameraMatrix * vec3_to_mat4(ptProj));
						
		unsigned short new_depth = ptProj(2) * FLOAT_TO_DEPTH;

		cv::Point2i pixLoc = cv::Point2i(ptProj2d) - voff;
		cv::Point2i pixLoc_off = pixLoc - offset + voff;

		if(CLAMP_SIZE(pixLoc_off.x, pixLoc_off.y, zBufferLocal.cols, zBufferLocal.rows)){
			zBufferLocal.at<unsigned short>(pixLoc_off) = new_depth;
		}
	}

	return zBufferLocal;
}

//should be in cylindermapping.cpp, but i dont like that file any more

PixelMap cylinderMapPixels(cv::Vec3f from_a, cv::Vec3f from_b, cv::Vec3f to_a, cv::Vec3f to_b, float radius, CroppedCvMat * fromMat, CroppedCvMat * toMat, cv::Size captureWindow, cv::Point captureOffset){
	
	
	float fromHeight = cv::norm(from_b-from_a);

	cv::Mat raycastTransform = segmentZeroTransformation(from_a, from_b);
	cv::Mat segmentTransform = segmentTransformation(from_a, from_b, to_a, to_b);

	std::vector<cv::Vec3f> fromPixels;
	std::vector<cv::Point2i> fromPixels_2d_v;

	//for(int r=0;r<fromMat->mat.rows;++r){
	//	for(int c=0;c<fromMat->mat.cols;++c){
	for(int r=0;r<captureWindow.height;++r){
		for(int c=0;c<captureWindow.width;++c){
			//if(fromMat->mat.at<cv::Vec3b>(r,c) == cv::Vec3b(255,0,0)){ //this one was used for "repair"
			if(true){
				//cv::Vec2f pt(c + fromMat->offset.x,
				//	r + fromMat->offset.y);
				cv::Vec2f pt(c + captureOffset.x,r + captureOffset.y);
				cv::Vec3f r_pt3 = raycast(pt, getInvCameraMatrix());

				cv::Vec3f pt3;
				if(rayCylinder2(raycastTransform, r_pt3, radius, fromHeight, &pt3)){
					fromPixels.push_back(pt3);
					fromPixels_2d_v.push_back(cv::Point2i(pt)-fromMat->offset);
				}
			}
		}
	}

	//this would be useful as a function maybe
	cv::Mat fromPixels_mat(4, fromPixels.size(), CV_32F, cv::Scalar(1));
	for(int i=0;i<fromPixels.size();++i){
		for(int j=0;j<3;++j)
			fromPixels_mat.at<float>(j, i) = fromPixels[i](j);
	}


	cv::Vec3f from_facing = cylinderFacingVector(from_a, from_b, 0);
	cv::Vec3f to_facing = cylinderFacingVector(to_a, to_b, 0);
	cv::Vec3f pre_to_facing = mat_to_vec(segmentTransform * vec3_to_mat4(from_a + from_facing));

	cv::Mat pre_rot_transform = segmentTransformation(to_a, pre_to_facing, to_a, to_a+to_facing);

	cv::Mat toPixels_mat = pre_rot_transform * segmentTransform * fromPixels_mat;
	//toPixels_mat = fromPixels_mat;

	cv::Mat toPixels_2d = getCameraMatrix() * toPixels_mat;
	std::vector<cv::Point2i> toPixels_2d_v;
	for(int i=0;i<toPixels_2d.cols;++i){
		cv::Point2i pt(mat4_to_vec2(toPixels_2d.col(i)));
		pt -= toMat->offset;
		toPixels_2d_v.push_back(pt);
	}

	return PixelMap(fromPixels_2d_v, toPixels_2d_v);
}

cv::Mat cylinderFacingTransform(cv::Vec3f a1, cv::Vec3f b1, float f1, cv::Vec3f a2, cv::Vec3f b2, float f2){
	
	cv::Mat segTrans = segmentTransformation(a1,b1,a2,b2);

	cv::Vec3f facing = cylinderFacingVector(a1,b1,f1);
	cv::Vec3f facing2 = cylinderFacingVector(a2,b2,f2);

	cv::Vec3f pre_facing2 = mat_to_vec(segTrans * vec3_to_mat4(a1 + facing));

	cv::Vec3f calc_axis = (facing2.cross(pre_facing2-a2));

	float angle = -acos(facing2.dot(pre_facing2-a2)/cv::norm(facing2)*cv::norm(pre_facing2-a2));

	cv::Mat rtrans = getTranslationMatrix(a2) * mat3_to_mat4(getRotationMatrix(calc_axis, angle)) * getTranslationMatrix(-a2);

	return rtrans * segTrans;
}

float tempCalcFacing(int limb, Skeleton s){
	if(limb == HEAD || limb == CHEST || limb == ABS){
		std::pair<int,int> p = KINECT::facingHelper(limb==ABS?2:1);
		cv::Mat a = s.points.col(p.first);
		cv::Mat b = s.points.col(p.second);

		cv::Vec3f perp = mat_to_vec(b-a);

		cv::Vec3f a2 = mat_to_vec(s.points.col(getLimbmap()[limb].first));
		cv::Vec3f b2 = mat_to_vec(s.points.col(getLimbmap()[limb].second));

		cv::Vec3f axis_facing = cylinderFacingVector(a2,b2,0);

		cv::Vec3f true_facing = (b2-a2).cross(perp);

		return axis_facing.dot(true_facing);

	}else
		return 0;
}

//TODO: fix this shitty argument list
PixelColorMap cylinderMapPixelsColor(cv::Vec3f from_a, cv::Vec3f from_b, float radius, int limbid, float facing, ScoreList scoreList, cv::Point voff, 
									 std::vector<SkeleVideoFrame> * vidRecord, CylinderBody * cylinderBody, Limbrary * limbrary, int blendMode){
	
	std::vector<cv::Vec3f> fromPixels;
	std::vector<cv::Vec3s> fromPixels_2d_v;

	//for(int r=0;r<fromMat->mat.rows;++r){
	//	for(int c=0;c<fromMat->mat.cols;++c){
	
	//calculateCylinderPoints(captureWindow, captureOffset, from_a, from_b, radius, fromMat->offset, &fromPixels, &fromPixels_2d_v);

	PixelPolygon p;

	cylinder_to_pts(from_a, from_b, radius, voff, &p, &fromPixels, &fromPixels_2d_v);

	//sort all the frames accdg to distance from the current skeleton


	int f = getLimbmap()[limbid].first;
	int s = getLimbmap()[limbid].second;

	std::vector<cv::Scalar> pixelColors;

	int erased = 0; //shitty hack

#if GH_DEBUG_CYLPROJ
	cv::Mat debugMat(captureWindow,CV_8UC3,cv::Scalar(255,255,255));
	cv::namedWindow("source mat", CV_WINDOW_NORMAL);

	std::vector<Segment3f> segments = cylinder_to_segments(from_a, from_b, radius, 16);

	for(auto it=segments.begin(); it!=segments.end(); ++it){
			
		cv::line(debugMat, cv::Point(toScreen(it->first))-captureOffset, cv::Point(toScreen(it->second))-captureOffset, cv::Scalar(255,255,0));
	}
#endif

	unsigned int blendLimit;

	switch(blendMode){
	case CMPC_NO_OCCLUSION:
		blendLimit = 1;
		break;
	case CMPC_BLEND_NONE:
		blendLimit = 1;
		break;
	case CMPC_BLEND_1:
		blendLimit = 5;
		break;
	}

	for(int i=0;i<fromPixels_2d_v.size();++i){
		
		std::vector<cv::Scalar> blended;

		for(auto it=scoreList.begin(); it!=scoreList.end(); ++it){

			CroppedCvMat texture;
			
			if(blendMode == CMPC_NO_OCCLUSION){
				texture = (*vidRecord)[it->first].videoFrame;
			}else{
				texture = (*limbrary).getFrameLimbs(it->first)[limbid];
			}

			cv::Vec3f _to_a = mat_to_vec((*vidRecord)[it->first].kinectPoints.points.col(f));
			cv::Vec3f _to_b = mat_to_vec((*vidRecord)[it->first].kinectPoints.points.col(s));

			cv::Vec3f to_a = _to_b + cylinderBody->newLeftOffset_cyl[limbid] * (_to_a - _to_b);
			cv::Vec3f to_b = _to_a + cylinderBody->newRightOffset_cyl[limbid] * (_to_b - _to_a);

			cv::Point2i pt = mapPixel
				(fromPixels[i+erased], texture.offset,
				from_a, from_b, tempCalcFacing(limbid, (*vidRecord)[it->first].kinectPoints), 
				to_a, to_b, facing);

			cv::Scalar pixelColor;

			int res = colorPixel(pt, limbid, texture, &pixelColor);
			
			if(res == CP_BG){
				blended.push_back(cv::Scalar(255,255,255));
			}else if(res == CP_GOOD || (blendMode == CMPC_NO_OCCLUSION && res != CP_OVER)){
				blended.push_back(pixelColor);
			} 

			if(blended.size() >= blendLimit) break;
		}
		
		if(blended.size() == 0){
			fromPixels_2d_v.erase(fromPixels_2d_v.begin() + i);
			--i;
			++erased;
		}else{
			//pixelColors.push_back(blended[0]);

			std::vector<float> blendAlpha(blended.size());
			std::fill(blendAlpha.begin(), blendAlpha.end(), 0);

			for(int i2=0;i2<blended.size();++i2){
				blendAlpha[i2] += 1.f/pow(2, i2+1);
			}
			blendAlpha[blended.size()-1] += 1.f/pow(2, blended.size());

			cv::Scalar blendPixel(0,0,0);

			float totalAlpha = 0;
			for(int i2=0;i2<blended.size();++i2){
				if(blended[i2] != cv::Scalar(255,255,255)){
					cv::Vec4b temp = blended[i2] * blendAlpha[i2];
					blendPixel += cv::Scalar(temp(0), temp(1), temp(2));
					totalAlpha += blendAlpha[i2];
				}
			}

			blendPixel *= 1.f/totalAlpha;

			//blend threshold; if alpha adds up to 0.5 or less, does not render the pixel
			if(totalAlpha > 0.5){
				pixelColors.push_back(blendPixel);
			}else{
				
				fromPixels_2d_v.erase(fromPixels_2d_v.begin() + i);
				--i;
				++erased;
			}
		}

		
	}

	return PixelColorMap(fromPixels_2d_v, pixelColors);
}

cv::Point2i mapPixel(cv::Vec3f pixLoc, cv::Point2i pixOffset, cv::Vec3f from_a, cv::Vec3f from_b, float from_facing, cv::Vec3f to_a, cv::Vec3f to_b, float to_facing){

	//cv::Mat segmentTransform = segmentTransformation(from_a, from_b, to_a, to_b);
	//cv::Vec3f to_facing = cylinderFacingVector(to_a, to_b, 0);
	//cv::Vec3f pre_to_facing = mat_to_vec(segmentTransform * vec3_to_mat4(from_a + from_facing));
	//
	//cv::Mat pre_rot_transform = segmentTransformation(to_a, pre_to_facing, to_a, to_a+to_facing);
	//
	//cv::Mat toPixels_mat = pre_rot_transform * segmentTransform * fromPixels_mat.col(i+erased);

	cv::Mat cylFacingTrans = cylinderFacingTransform(from_a, from_b, from_facing, to_a, to_b, to_facing);

	cv::Mat toPixels_mat = cylFacingTrans * vec3_to_mat4(pixLoc);

	cv::Mat toPixels_2d = getCameraMatrix() * toPixels_mat;

	cv::Point2i pt(mat4_to_vec2(toPixels_2d));

	pt -= pixOffset;

	return pt;
}

int colorPixel(cv::Point2i pt, int limbid, CroppedCvMat texture, cv::Scalar * pixelColor){


	if(CLAMP_SIZE(pt.x, pt.y, texture.mat.cols, texture.mat.rows)){

#if GH_DEBUG_CYLPROJ
		cv::Point debugPt(fromPixels_2d_v[i](0), fromPixels_2d_v[i](1));
		debugPt += fromMat->offset - captureOffset;
		debugMat.at<cv::Vec3b>(debugPt) = cv::Vec3b(0,0,255);

		cv::Mat sourceMat = texture.mat.clone();

		std::vector<Segment3f> segments = cylinder_to_segments(to_a, to_b, radius, 16);

		for(auto it2=segments.begin(); it2!=segments.end(); ++it2){
			
			cv::line(sourceMat, 
				cv::Point(toScreen(it2->first))-texture.offset, 
				cv::Point(toScreen(it2->second))-texture.offset, 
				cv::Scalar(255,255,0));
		}

		cv::rectangle(sourceMat,pt-cv::Point(1,1),pt+cv::Point(1,1),cv::Scalar(0,0,255));
						
		cv::imshow("debug mat", debugMat);
		cv::imshow("source mat", sourceMat);
		cv::waitKey();
#endif
		*pixelColor = (cv::Scalar(texture.mat.at<cv::Vec3b>(pt)));

		if(texture.mat.at<cv::Vec3b>(pt) != cv::Vec3b(255,0,0)){
			if(texture.mat.at<cv::Vec3b>(pt) != cv::Vec3b(255,255,255)){
#if GH_DEBUG_CYLPROJ
				debugMat.at<cv::Vec3b>(debugPt) = texture.mat.at<cv::Vec3b>(pt);
				//sourceMat.at<cv::Vec3b>(pt) = cv::Vec3b(0,0,255);
						
				cv::imshow("debug mat", debugMat);
				cv::imshow("source mat", sourceMat);
				cv::waitKey();
#endif

				return CP_GOOD;
			}else{
				return CP_BG;
			}
		}else{
			return CP_OCCLUDED;
		}
	}else{
		return CP_OVER;
	}

}