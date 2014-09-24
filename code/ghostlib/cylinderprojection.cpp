#include "cylinderprojection.h"
#include "cylinderintersection.h"
#include "cvutil.h"
#include "bodybuild.h"
#include "ghostcam.h"
#include "KinectManager.h"

#include "camlerp.h"

//parallelization
//#include <ppl.h>
//#include "parallel_projection.h"



cv::Mat cylinder_to_pts(unsigned int imgWidth, unsigned int imgHeight, cv::Vec3f a_, cv::Vec3f b_, float radius, cv::Point voff, PixelPolygon * p, std::vector<cv::Vec3f> * fromPixels, std::vector<cv::Vec3s> * fromPixels_2d_v){
	
	cv::Mat ret(4, 0, CV_32F, cv::Scalar(1));

	cv::Vec3f a,b;

	if(cv::norm(a_) < cv::norm(b_)){
		a = a_;
		b = b_;
	}else{
		a = b_;
		b = a_;
	}

	//std::vector<cv::Vec3f> pts = cylinder_to_vertices(a, b, radius,8);
	//std::vector<cv::Vec2f> pts2 = vec3f_to_2f(pts, cv::Vec2f(voff.x, voff.y));
	//*p = polygon_contains_pixels(pts2);

	std::vector<Segment3f> pts = cylinder_to_segments(a, b, radius,8);
	std::vector<Segment2f> pts2 = segment3f_to_2f(pts, cv::Vec2f(voff.x, voff.y));
	if(pts2.empty()) return ret;
	*p = polygon_contains_pixels(pts2);
			
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

	int limbpicWidth = p->hi.size();
	int limbpicHeight = p->hi_y-p->lo_y;
	
	//std::cout << p->x_offset << " " << limbpicWidth << " " << p->lo_y << " " << limbpicHeight << std::endl;
	cv::Mat rayMat(4, limbpicHeight*limbpicWidth, CV_32F);

#if 1

	cv::Rect boundingBox(0,0,imgWidth,imgHeight);
	LerpCorners lc = generateLerpCorners(boundingBox);
	
	for(int _x = 0; _x < limbpicWidth; ++_x){
		for(int _y = 0; _y < limbpicHeight; ++_y){
			int y = _y + p->lo_y;
			int x = _x + p->x_offset;
			cv::Vec3f ray = lerpPoint(x+voff.x,y+voff.y,boundingBox,lc);

			rayMat.ptr<float>(0)[_y*limbpicWidth+_x] = ray(0);
			rayMat.ptr<float>(1)[_y*limbpicWidth+_x] = ray(1);
			rayMat.ptr<float>(2)[_y*limbpicWidth+_x] = 1;
			rayMat.ptr<float>(3)[_y*limbpicWidth+_x] = 1;

		}
	}

#else

	cv::Mat pts2D(2, limbpicWidth * limbpicHeight, CV_32F);
	for(int i=0;i<limbpicWidth * limbpicHeight;++i){
		int _x = i%limbpicWidth;
		int _y = i/limbpicWidth;
		int x = _x + p->x_offset+voff.x;
		int y = _y + p->lo_y+voff.y;
		pts2D.ptr<float>(0)[i] = x;
		pts2D.ptr<float>(1)[i] = y;
	}

	rayMat = KINECT::makeRays(pts2D);

#endif

	rayMat = transformation * rayMat;

	for(int _x = 0; _x < limbpicWidth; ++_x){
		for(int _y = 0; _y < limbpicHeight; ++_y){
			int y = _y + p->lo_y;
			int x = _x + p->x_offset;

			//cv::Vec3f pt(x+voff.x,y+voff.y,1);
			//cv::Vec3f ray = mat_to_vec(invCameraMatrix * cv::Mat(pt)); replaced with lerp
			//cv::Vec3f ray = lerpPoint(x+voff.x,y+voff.y,boundingBox,lc);
			//cv::Mat ray_trans1 = transformation * vec3_to_mat4(ray);
			cv::Mat ray_trans = rayMat.col(_y*limbpicWidth+_x).clone();
			cv::Vec3f ptProj;

			//int res = rayCylinder2(transformation, ray, radius, height, &ptProj);
			int res = rayCylinder3_c(origin_trans, ray_trans.ptr<float>(), transformation_inv, C, height, &ptProj);
			if(res == 1){
				fromPixels->push_back(ptProj);
				//cv::Vec3s xyDepth(pt(0)-voff.x, pt(1)-voff.y, ptProj(2) * FLOAT_TO_DEPTH);
				cv::Vec3s xyDepth(x, y, ptProj(2) * FLOAT_TO_DEPTH);
				fromPixels_2d_v->push_back(xyDepth);
			}
		}
	}

	ret.create(4,fromPixels->size(),CV_32F);
	for(int j=0;j<4;++j){
		float * retptr = ret.ptr<float>(j);
		for(int i=0;i<fromPixels->size();++i){
			//ret.at<float>(j,i) = (*fromPixels)[i](j);
			if(j<3)
				*(retptr+i) = (*fromPixels)[i](j);
			else
				*(retptr+i) = 1;
		}
	}
	

	return ret;
}


//copy of cylinder_to_pts that involves a cv::Rect instead of a PixelPolygon
cv::Mat cylinder_to_pts(unsigned int imgWidth, unsigned int imgHeight, cv::Vec3f a_, cv::Vec3f b_, float radius, cv::Point voff, cv::Rect * r, std::vector<cv::Vec3f> * fromPixels, std::vector<cv::Vec3s> * fromPixels_2d_v){
	
	cv::Mat ret(4, 0, CV_32F, cv::Scalar(1));
	cv::Vec3f a,b;

	if(a_(2) < b_(2)){
		a = a_;
		b = b_;
	}else{
		a = b_;
		b = a_;
	}

	//std::vector<cv::Vec3f> pts = cylinder_to_vertices(a, b, radius,8);
	//std::vector<cv::Vec2f> pts2 = vec3f_to_2f(pts, cv::Vec2f(voff.x, voff.y));
	//*p = polygon_contains_pixels(pts2);

	std::vector<Segment3f> pts = cylinder_to_segments(a, b, radius,8);
	std::vector<Segment2f> pts2 = segment3f_to_2f(pts, cv::Vec2f(voff.x, voff.y));
	if(pts2.empty()) return ret;
	*r = cv::boundingRect(segments_to_points(pts2));
			
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

	int limbpicWidth = r->width;
	int limbpicHeight = r->height;
	
	//std::cout << p->x_offset << " " << limbpicWidth << " " << p->lo_y << " " << limbpicHeight << std::endl;

	cv::Rect boundingBox(0,0,imgWidth,imgHeight);
	LerpCorners lc = generateLerpCorners(boundingBox);
	
	cv::Mat rayMat(4, limbpicHeight*limbpicWidth, CV_32F);

	for(int _x = 0; _x < limbpicWidth; ++_x){
		for(int _y = 0; _y < limbpicHeight; ++_y){
			int y = _y + r->y;
			int x = _x + r->x;
			cv::Vec3f ray = lerpPoint(x+voff.x,y+voff.y,boundingBox,lc);

			rayMat.ptr<float>(0)[_y*limbpicWidth+_x] = ray(0);
			rayMat.ptr<float>(1)[_y*limbpicWidth+_x] = ray(1);
			rayMat.ptr<float>(2)[_y*limbpicWidth+_x] = 1;
			rayMat.ptr<float>(3)[_y*limbpicWidth+_x] = 1;

		}
	}

	rayMat = transformation * rayMat;

	for(int _x = 0; _x < limbpicWidth; ++_x){
		for(int _y = 0; _y < limbpicHeight; ++_y){
			int y = _y + r->y;
			int x = _x + r->x;

			//cv::Vec3f pt(x+voff.x,y+voff.y,1);
			//cv::Vec3f ray = mat_to_vec(invCameraMatrix * cv::Mat(pt)); replaced with lerp
			//cv::Vec3f ray = lerpPoint(x+voff.x,y+voff.y,boundingBox,lc);
			//cv::Mat ray_trans1 = transformation * vec3_to_mat4(ray);
			cv::Mat ray_trans = rayMat.col(_y*limbpicWidth+_x).clone();
			cv::Vec3f ptProj;

			//int res = rayCylinder2(transformation, ray, radius, height, &ptProj);
			int res = rayCylinder3_c(origin_trans, ray_trans.ptr<float>(), transformation_inv, C, height, &ptProj);
			if(res == 1){
				fromPixels->push_back(ptProj);
				//cv::Vec3s xyDepth(pt(0)-voff.x, pt(1)-voff.y, ptProj(2) * FLOAT_TO_DEPTH);
				cv::Vec3s xyDepth(x, y, ptProj(2) * FLOAT_TO_DEPTH);
				fromPixels_2d_v->push_back(xyDepth);
			}
		}
	}

	ret.create(4,fromPixels->size(),CV_32F);
	for(int j=0;j<4;++j){
		float * retptr = ret.ptr<float>(j);
		for(int i=0;i<fromPixels->size();++i){
			//ret.at<float>(j,i) = (*fromPixels)[i](j);
			if(j<3)
				*(retptr+i) = (*fromPixels)[i](j);
			else
				*(retptr+i) = 1;
		}
	}
	

	return ret;
}

#define CYLAPPROXPOINT 8


#if 0
//NOTE: this version of cylinder_to_pts has a BIG BUG
//depth values are WRONG; fix

cv::Mat cylinder_to_pts(cv::Vec3f a_, cv::Vec3f b_, float radius, cv::Point voff, std::vector<cv::Vec3f> * fromPixels, std::vector<cv::Vec3s> * fromPixels_2d_v){

	cv::Vec3f a,b;

	if(a_(2) < b_(2)){
		a = a_;
		b = b_;
	}else{
		a = b_;
		b = a_;
	}

	int xmin=voff.x+1, ymin=voff.y+1, xmax=WIDTH-1, ymax=HEIGHT-1;

	cv::Vec3f cross(0, radius*2, 0);
	cv::Vec3f cross2(radius*2, 0, 0);

	cv::Vec3f vecs[CYLAPPROXPOINT];

	vecs[0] = a + (cross);
	vecs[1] = a - (cross);
	vecs[2] = b + (cross);
	vecs[3] = b - (cross);
	vecs[4] = a + (cross2);
	vecs[5] = a - (cross2);
	vecs[6] = b + (cross2);
	vecs[7] = b - (cross2);

	cv::Mat vecm(4,CYLAPPROXPOINT,CV_32F);
	for(int i=0;i<4;++i){
		float * vecmptr = vecm.ptr<float>(i);
		if(i<3){
			for(int j=0;j<CYLAPPROXPOINT;++j){
				*(vecmptr+j) = vecs[j](i);
			}
		}else{
			for(int j=0;j<CYLAPPROXPOINT;++j){
				*(vecmptr+j) = 1;
			}
		}
	}

	cv::Mat vecm2d = getCameraMatrix() * vecm;

	for(int i=0;i<CYLAPPROXPOINT;++i){
		float z = *(vecm2d.ptr<float>(2)+i);
		float x = (*(vecm2d.ptr<float>(0)+i)/z);
		float y = (*(vecm2d.ptr<float>(1)+i)/z);

		if(x < xmin) xmin = x;
		if(x > xmax) xmax = x;
		if(y < ymin) ymin = y;
		if(y > ymax) ymax = y;
	}

	xmin -= voff.x;
	xmax -= voff.x;
	ymin -= voff.y;
	ymax -= voff.y;

	//transform the space:
	cv::Vec3f cyl_axis = b - a;

	float height = cv::norm(cyl_axis);
	cv::Mat transformation_inv;
	cv::Mat transformation = segmentZeroTransformation(a,b,&transformation_inv);

	//cv::Vec3f origin_trans = mat_to_vec(transformation.col(3)); //mat_to_vec3(transformation * vec3_to_mat4(cv::Vec3f(0,0,0)));
	float origin_trans[3];
	origin_trans[0] = *(transformation.ptr<float>(0)+3);
	origin_trans[1] = *(transformation.ptr<float>(1)+3);
	origin_trans[2] = *(transformation.ptr<float>(2)+3);

	int pwidth = xmax - xmin;
	int pheight = ymax - ymin;

	//std::cout << xmin << " " << pwidth << " " << ymin << " " <<  pheight << std::endl;


	int psize = pwidth*pheight;

	cv::Mat points(4, psize, CV_32F);

	for(int _x = 0; _x < pwidth; ++_x){
		for(int _y = 0; _y < pheight; ++_y){
			int y = _y + ymin;
			int x = _x + xmin;

			//cv::Vec3f pt(x+voff.x,y+voff.y,1);
			//cv::Vec3f ray = mat_to_vec(invCameraMatrix * cv::Mat(pt)); replaced with lerp
			cv::Vec3f ray = lerpPoint(x+voff.x,y+voff.y,lerpBoundingBox,lerpCorners);
			
			for(int c=0;c<3;++c){
				points.ptr<float>(c)[_y*pwidth+_x] = ray(c);
			}
			points.ptr<float>(3)[_y*pwidth+_x] = 1;

			//int i = _y*pwidth+_x;
			//points.ptr<float>(0)[i] = x;
			//points.ptr<float>(1)[i] = y;
			//points.ptr<float>(2)[i] = 1;
			//points.ptr<float>(3)[i] = 1;

		}
	}

	//points = mat3_to_mat4(invCameraMatrix) * points;

	*(transformation.ptr<float>(0)+3) = 0;
	*(transformation.ptr<float>(1)+3) = 0;
	*(transformation.ptr<float>(2)+3) = 0;

	cv::Mat points_trans = transformation * points;

	float C = origin_trans[0]*origin_trans[0] + origin_trans[1]*origin_trans[1] -radius*radius;

	//cv::Mat A_sq = points.t() * points;

	for(int i=0;i<psize;++i){
		int _y = i/pwidth;
		int _x = i%pwidth;

		int y = _y + ymin;
		int x = _x + xmin;
		float ray_trans[3] = {points_trans.ptr<float>(0)[i], points_trans.ptr<float>(1)[i], points_trans.ptr<float>(2)[i]};

#if 0
		//int res = rayCylinder2(transformation, ray, radius, height, &ptProj);

		cv::Vec3f ptProj;
		int res = rayCylinder3(origin_trans, ray_trans, transformation_inv, radius, height, &ptProj);
		if(res == 1){
			fromPixels->push_back(ptProj);
			//cv::Vec3s xyDepth(pt(0)-voff.x, pt(1)-voff.y, ptProj(2) * FLOAT_TO_DEPTH);
			cv::Vec3s xyDepth(x, y, ptProj(2) * FLOAT_TO_DEPTH);
			fromPixels_2d_v->push_back(xyDepth);
		}

		//float retf[4];
		//bool res = rayCylinderClosestIntersectionPoint(origin_trans, ray_trans, radius, height, retf);

#elseif 1
		float A = ray_trans[0]*ray_trans[0] + ray_trans[1]*ray_trans[1];
		//float A = A_sq.ptr<float>(i)[i] - 2;
		float B = 2*origin_trans[0]*ray_trans[0]+2*origin_trans[1]*ray_trans[1];

		float sqrtval = sqrt(B*B-4*A*C);

		float minus = (-B - sqrtval)/(2*A);

		float vminus2 = origin_trans[2]+minus*ray_trans[2];

		bool res = false;
		float retf[4];

		if(vminus2 > 0 && vminus2 < height)
		{

			retf[0] = origin_trans[0]+minus*ray_trans[0];
			retf[1] = origin_trans[1]+minus*ray_trans[1];
			retf[2] = origin_trans[2]+minus*ray_trans[2];

			res = true;
		}
		else{
			//float plus = (-B + sqrtval)/(2*A);
			//float vplus2 =  origin_trans[2]+plus*ray_trans[2];
			//if( (vplus2 <= 0 && vminus2 > 0) || (vminus2 <= 0 && vplus2 > 0)){
			//	float extra = -origin_trans[2]/ray_trans[2];
			//	retf[0] = origin_trans[0]+extra*ray_trans[0];
			//	retf[1] = origin_trans[1]+extra*ray_trans[1];
			//	retf[2] = origin_trans[2]+extra*ray_trans[2];
			//	res = true;
			//}
		}

		if(res){
			retf[3] = 1;
			//ptProj = mat_to_vec3(transformation_inv*cv::Mat(4,1,CV_32F,retf));
			cv::Vec3f ptProj(retf[0], retf[1], retf[2]);
			fromPixels->push_back(ptProj);
			//cv::Vec3s xyDepth(pt(0)-voff.x, pt(1)-voff.y, ptProj(2) * FLOAT_TO_DEPTH);
			cv::Vec3s xyDepth(x, y, ptProj(2) * FLOAT_TO_DEPTH);
			fromPixels_2d_v->push_back(xyDepth);
		}
#endif
	}

	cv::Mat ret(4, fromPixels->size(), CV_32F);

	if(!fromPixels->empty()){
		for(int i=0;i<fromPixels->size();++i){
			//for(int j=0;j<3;++j){
			//	float * retptr = ret.ptr<float>(j);
			//	//ret.at<float>(j,i) = (*fromPixels)[i](j);
			//	ret.ptr<float>(j)[i] = (*fromPixels)[i](j);
			//}
			
			ret.ptr<float>(0)[i] = (*fromPixels)[i](0);
			ret.ptr<float>(1)[i] = (*fromPixels)[i](1);
			ret.ptr<float>(2)[i] = (*fromPixels)[i](2);
			ret.ptr<float>(3)[i] = 1;
		}
	}

	return transformation_inv * ret;
}

#elseif 0

//actually this is really slow, idk why

cv::Mat cylinder_to_pts(cv::Vec3f a_, cv::Vec3f b_, float radius, cv::Point voff, std::vector<cv::Vec3f> * fromPixels, std::vector<cv::Vec3s> * fromPixels_2d_v){
	
	cv::Mat ret(4, 0, CV_32F, cv::Scalar(1));
	cv::Vec3f a,b;

	if(a_(2) < b_(2)){
		a = a_;
		b = b_;
	}else{
		a = b_;
		b = a_;
	}

	int xmin=voff.x+1, ymin=voff.y+1, xmax=WIDTH-1, ymax=HEIGHT-1;

	cv::Vec3f cross(0, radius*2, 0);
	cv::Vec3f cross2(radius*2, 0, 0);

	cv::Vec3f vecs[CYLAPPROXPOINT];

	vecs[0] = a + (cross);
	vecs[1] = a - (cross);
	vecs[2] = b + (cross);
	vecs[3] = b - (cross);
	vecs[4] = a + (cross2);
	vecs[5] = a - (cross2);
	vecs[6] = b + (cross2);
	vecs[7] = b - (cross2);

	cv::Mat vecm(4,CYLAPPROXPOINT,CV_32F);
	for(int i=0;i<4;++i){
		float * vecmptr = vecm.ptr<float>(i);
		if(i<3){
			for(int j=0;j<CYLAPPROXPOINT;++j){
				*(vecmptr+j) = vecs[j](i);
			}
		}else{
			for(int j=0;j<CYLAPPROXPOINT;++j){
				*(vecmptr+j) = 1;
			}
		}
	}

	cv::Mat vecm2d = getCameraMatrix() * vecm;

	for(int i=0;i<CYLAPPROXPOINT;++i){
		float z = *(vecm2d.ptr<float>(2)+i);
		float x = (*(vecm2d.ptr<float>(0)+i)/z);
		float y = (*(vecm2d.ptr<float>(1)+i)/z);

		if(x < xmin) xmin = x;
		if(x > xmax) xmax = x;
		if(y < ymin) ymin = y;
		if(y > ymax) ymax = y;
	}

	xmin -= voff.x;
	xmax -= voff.x;
	ymin -= voff.y;
	ymax -= voff.y;

	int limbpicWidth = xmax - xmin;
	int limbpicHeight = ymax - ymin;
	
	cv::Vec3f cyl_axis = b - a;

	float height = cv::norm(cyl_axis);
	cv::Mat transformation = segmentZeroTransformation(a,b);
	cv::Mat transformation_inv = transformation.inv();

	//cv::Vec3f origin_trans = mat_to_vec(transformation.col(3)); //mat_to_vec3(transformation * vec3_to_mat4(cv::Vec3f(0,0,0)));
	float origin_trans[3];
	for(int i=0;i<3;++i){
		origin_trans[i] = *(transformation.ptr<float>(i)+3);
	}

	cv::Rect boundingBox(0,0,WIDTH,HEIGHT);
	LerpCorners lc = generateLerpCorners(boundingBox);

	for(int _x = 0; _x < limbpicWidth; ++_x){
		for(int _y = 0; _y < limbpicHeight; ++_y){
			int y = _y + ymin;
			int x = _x + xmin;

			//cv::Vec3f pt(x+voff.x,y+voff.y,1);
			//cv::Vec3f ray = mat_to_vec(invCameraMatrix * cv::Mat(pt)); replaced with lerp
			cv::Vec3f ray = lerpPoint(x+voff.x,y+voff.y,boundingBox,lc);
			cv::Mat ray_trans = transformation * vec3_to_mat4(ray);
			cv::Vec3f ptProj;

			//int res = rayCylinder2(transformation, ray, radius, height, &ptProj);
			int res = rayCylinder3(origin_trans, ray_trans.ptr<float>(), transformation_inv, radius, height, &ptProj);
			if(res == 1){
				fromPixels->push_back(ptProj);
				//cv::Vec3s xyDepth(pt(0)-voff.x, pt(1)-voff.y, ptProj(2) * FLOAT_TO_DEPTH);
				cv::Vec3s xyDepth(x, y, ptProj(2) * FLOAT_TO_DEPTH);
				fromPixels_2d_v->push_back(xyDepth);
			}
		}
	}

	ret.create(4,fromPixels->size(),CV_32F);
	for(int j=0;j<4;++j){
		float * retptr = ret.ptr<float>(j);
		for(int i=0;i<fromPixels->size();++i){
			//ret.at<float>(j,i) = (*fromPixels)[i](j);
			if(j<3)
				*(retptr+i) = (*fromPixels)[i](j);
			else
				*(retptr+i) = 1;
		}
	}
	

	return ret;
}
#endif

//2nd version
cv::Mat pts_to_zBuffer(std::vector<cv::Vec3s>& cylPts, cv::Point offset, unsigned int width, unsigned int height){

	cv::Mat zBufferLocal(height, width, CV_16U, cv::Scalar(MAXDEPTH));
	
	for (int i = 0; i<cylPts.size(); ++i){
		cv::Point2i pixLoc(cylPts[i](0), cylPts[i](1));
		cv::Point2i pixLoc_off = pixLoc - offset;

		if (CLAMP_SIZE(pixLoc_off.x, pixLoc_off.y, zBufferLocal.cols, zBufferLocal.rows)){
			zBufferLocal.at<unsigned short>(pixLoc_off) = cylPts[i](2);
		}
	}

	return zBufferLocal;
}

//first version
cv::Mat pts_to_zBuffer(cv::Mat cylPts, cv::Point voff, cv::Point offset, unsigned int width, unsigned int height){

	cv::Mat zBufferLocal(height, width, CV_16U, cv::Scalar(MAXDEPTH));
#if GHOST_CAPTURE == CAPTURE_OPENNI
	cv::Mat cameraMatrix = getCameraMatrixScene();
#elif GHOST_CAPTURE == CAPTURE_KINECT2
	//cv::Mat convertedDepthPoints = KINECT::mapCameraPointsToColorPoints(cylPts);
	cv::Mat convertedDepthPoints = getCameraMatrix()*cylPts;
#endif

	for(int i=0;i<cylPts.cols;++i){
#if GHOST_CAPTURE == CAPTURE_OPENNI
		cv::Vec3f ptProj = mat_to_vec3(cylPts.col(i));
		cv::Vec2f ptProj2d = mat4_to_vec2(cameraMatrix * vec3_to_mat4(ptProj));
		unsigned short new_depth = ptProj(2) * FLOAT_TO_DEPTH;
#elif GHOST_CAPTURE == CAPTURE_KINECT2
		cv::Vec2f ptProj2d(
			convertedDepthPoints.ptr<float>(0)[i]/
			convertedDepthPoints.ptr<float>(2)[i],
			convertedDepthPoints.ptr<float>(1)[i] /
			convertedDepthPoints.ptr<float>(2)[i]);
		unsigned short new_depth = cylPts.ptr<float>(2)[i] * FLOAT_TO_DEPTH;
#endif
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
				cv::Vec3f r_pt3 = raycast(pt, getInvCameraMatrixScene());

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
		for(int j=0;j<3;++j){
			fromPixels_mat.at<float>(j, i) = fromPixels[i](j);
		}
	}


	cv::Vec3f from_facing = cylinderFacingVector(from_a, from_b, 0);
	cv::Vec3f to_facing = cylinderFacingVector(to_a, to_b, 0);
	cv::Vec3f pre_to_facing = mat_to_vec3(segmentTransform * vec3_to_mat4(from_a + from_facing));

	cv::Mat pre_rot_transform = segmentTransformation(to_a, pre_to_facing, to_a, to_a+to_facing);

	cv::Mat toPixels_mat = pre_rot_transform * segmentTransform * fromPixels_mat;
	//toPixels_mat = fromPixels_mat;

	cv::Mat toPixels_2d = getCameraMatrixTexture() * toPixels_mat;
	std::vector<cv::Point2i> toPixels_2d_v;
	for(int i=0;i<toPixels_2d.cols;++i){
		cv::Point2i pt(mat4_to_vec2(toPixels_2d.col(i)));
		pt -= toMat->offset;
		toPixels_2d_v.push_back(pt);
	}

	return PixelMap(fromPixels_2d_v, toPixels_2d_v);
}

cv::Mat cylinderFacingTransform(cv::Vec3f a1, cv::Vec3f b1, float f1, cv::Vec3f a2, cv::Vec3f b2, float f2, float r){
	
	cv::Mat segTrans_r = segmentTransformation(a1,b1,a2,b2,r);
	cv::Mat segTrans = segmentTransformation(a1,b1,a2,b2);

	cv::Vec3f facing = cylinderFacingVector(a1,b1,f1);
	cv::Vec3f facing2 = cylinderFacingVector(a2,b2,f2);

	cv::Vec3f pre_facing2 = mat_to_vec3(segTrans * vec3_to_mat4(a1 + facing));

	cv::Vec3f calc_axis = (facing2.cross(pre_facing2-a2));

	float angle = -acos(facing2.dot(pre_facing2-a2)/cv::norm(facing2)*cv::norm(pre_facing2-a2));

	cv::Mat rtrans = getTranslationMatrix(a2) * (getRotationMatrix4(calc_axis, angle)) * getTranslationMatrix(-a2);

	return rtrans * segTrans_r;
}


cv::Mat cylinderFacingTransform2(cv::Vec3f a1, cv::Vec3f b1, float f1, cv::Vec3f a2, cv::Vec3f b2, float f2){
	cv::Mat F(4,5,CV_32F);
	cv::Mat T(4,5,CV_32F);

	cv::Vec3f facing = cylinderFacingVector(a1,b1,f1);
	cv::Vec3f facing2 = cylinderFacingVector(a2,b2,f2);

	cv::Vec3f f1_1 = a1 + facing;
	cv::Vec3f f2_1 = a1 - facing;
	cv::Vec3f f3_1 = b1 + facing;
	
	cv::Vec3f f1_2 = a2 + facing2;
	cv::Vec3f f2_2 = a2 - facing2;
	cv::Vec3f f3_2 = b2 + facing2;

	for(int i=0;i<3;++i){
		float * Fptr = F.ptr<float>(i);
		Fptr[0] = a1(i);
		Fptr[1] = b1(i);
		Fptr[2] = f1_1(i);
		Fptr[3] = f2_1(i);
		Fptr[4] = f2_1(i);
		float * Tptr = T.ptr<float>(i);
		Tptr[0] = a2(i);
		Tptr[1] = b2(i);
		Tptr[2] = f1_2(i);
		Tptr[3] = f2_2(i);
		Tptr[4] = f2_2(i);
	}

	cv::Mat F_inv;
	cv::invert(F, F_inv, cv::DECOMP_SVD);
	cv::Mat M = T * F_inv;

	return M;
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

	cv::Mat toPixels_2d = getCameraMatrixTexture() * toPixels_mat;

	cv::Point2i pt(mat4_to_vec2(toPixels_2d));

	pt -= pixOffset;

	return pt;
}


//util

