#include "ghostdraw.h"
#include "definitions.h"
#include "cvutil.h"
#include "cylinderprojection.h"
#include "cylinderintersection.h"
#include "drawing.h"
#include "bodybuild.h"
#include "ghostcam.h"
#include "texturesearch.h"
#include "Limbrary.h"
#include "CylinderBody.h"


void ghostdraw(int frame, cv::Mat transform, std::vector<SkeleVideoFrame>& vidRecord, std::vector<Skeleton>& wcSkeletons, CylinderBody& cylinderBody, Limbrary& limbrary, cv::Mat draw, unsigned char options){
	cv::Mat zBuf(480, 640, CV_16U, cv::Scalar(MAXDEPTH));

	for(int i=0;i<NUMLIMBS;++i){

		int f = getLimbmap()[i].first;
		int s = getLimbmap()[i].second;

		cv::Vec3f _a = mat_to_vec(wcSkeletons[frame].points.col(f));
		cv::Vec3f _b = mat_to_vec(wcSkeletons[frame].points.col(s));

		cv::Vec3f __a = _b + cylinderBody.newLeftOffset_cyl[i] * (_a - _b);
		cv::Vec3f __b = _a + cylinderBody.newRightOffset_cyl[i] * (_b - _a);

		cv::Vec3f a = mat_to_vec(transform * vec3_to_mat4(__a));
		cv::Vec3f b = mat_to_vec(transform * vec3_to_mat4(__b));

		float radius = cylinderBody.newPartRadii_cyl[i];

		if(options & GD_DRAW)
		{

			//wtf is this
			//int bestFrame = 100;
			//
			////pixel color mapping
			//cv::Vec3f _from_a = mat_to_vec(vidRecord[bestFrame].kinectPoints.points.col(f));
			//cv::Vec3f _from_b = mat_to_vec(vidRecord[bestFrame].kinectPoints.points.col(s));
			//
			//cv::Vec3f from_a = _from_b + cylinderBody.newLeftOffset_cyl[i] * (_from_a-_from_b);
			//cv::Vec3f from_b = _from_a + cylinderBody.newRightOffset_cyl[i] * (_from_b-_from_a);
			//
			//
			//CroppedCvMat dest = limbrary.getFrameLimbs(bestFrame)[i];
			CroppedCvMat source = limbrary.getFrameLimbs(frame)[i];
#if 0
			//weird shiz with from and to
			//basically the transformation is going FROM a-b to from_a-from_b
			//but the colors are going FROM from_a-from_b vice versa
			//~from now on managers are willing to add are "Rest"
			PixelMap from_to = cylinderMapPixels(a, b, from_a, from_b, radius, &source, &dest, 
				cv::Size(p.hi.size(), p.hi_y-p.lo_y), cv::Point(p.x_offset, p.lo_y));


			

			//for(int _x=0; _x<p.lo.size(); ++_x){
			//	for(int y=p.lo[_x]; y<p.hi[_x]; ++y){
			//		int x = _x + p.x_offset;
			//
			//		//cvDrawPoint(draw, cv::Point(x,y), getLimbColor(i));
			//
			//
			//	}
			//}

			for(int j=0; j<from_to.first.size(); ++j){
				
				if(CLAMP_SIZE(from_to.second[j].x, from_to.second[j].y, dest.mat.cols,dest.mat.rows )){
					
					if(dest.mat.at<cv::Vec3b>(from_to.second[j]) != cv::Vec3b(255,0,0) && 
						dest.mat.at<cv::Vec3b>(from_to.second[j]) != cv::Vec3b(255,255,255)){
						//source.mat.at<cv::Vec3b>(from_to.first[i]) = dest.mat.at<cv::Vec3b>(from_to.second[i]);
						cvDrawPoint(draw, from_to.first[j] + source.offset, cv::Scalar(dest.mat.at<cv::Vec3b>(from_to.second[j])));
					}
				}
			}
#else
			Skeleton skele = vidRecord[frame].kinectPoints;
			skele.points = transform * skele.points;
			
			float facing = tempCalcFacing(i, skele);
			ScoreList scoreList = sortFrames(skele, &vidRecord, &limbrary, i, TEXTURE_SEARCH_DEPTH, true);
			PixelColorMap from_color = cylinderMapPixelsColor(a, b, radius, i, facing, scoreList, &source, 
				&vidRecord, &cylinderBody, &limbrary, CMPC_BLEND_1);

			for(int j=0;j<from_color.first.size();++j){

				cv::Point pt(from_color.first[j](0), from_color.first[j](1));
				pt += source.offset;

				unsigned short new_depth = from_color.first[j](2);
				unsigned short old_depth = zBuf.at<unsigned short>(pt);

				if(new_depth < old_depth && new_depth > 0){

					cvDrawPoint(draw, pt, from_color.second[j]);
					zBuf.at<unsigned short>(pt) = new_depth;
				
					//cv::imshow("pic", draw);
					//cv::waitKey(10);
				}
			}
#endif

		}

		if(options & GD_CYL){

			std::vector<Segment3f> segments = cylinder_to_segments(a, b, radius, 16);

			for(auto it=segments.begin(); it!=segments.end(); ++it){
			
				cv::line(draw, cv::Point(toScreen(it->first)), cv::Point(toScreen(it->second)), cv::Scalar(255,255,0));
			}
		}
	}
}