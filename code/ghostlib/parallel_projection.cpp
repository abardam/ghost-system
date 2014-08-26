#include "parallel_projection.h"
#include "cylinderprojection.h"
#include "ghostmapping.h"
#include "cvutil.h"

parallel_pixelMapping::parallel_pixelMapping(std::vector<SkeleVideoFrame> * vidRecord,
	CylinderBody * cylinderBody,
	Limbrary * limbrary,
	ScoreList * scoreList,

	cv::Vec3f * fromPixels,
	cv::Scalar * pixelColors,
	std::vector<bool> * erasevector,

	cv::Vec3f from_a, cv::Vec3f from_b,
	int limbid,
	float facing,
	float radius,

	int blendMode,
	int blendLimit){
		this->vidRecord = vidRecord;
		this->cylinderBody = cylinderBody;
		this->limbrary = limbrary;
		this->scoreList = scoreList;
		this->fromPixels = fromPixels;
		this->pixelColors = pixelColors;
		this->erasevector = erasevector;
		this->from_a = from_a;
		this->from_b = from_b;
		this->limbid = limbid;
		this->facing = facing;
		this->radius = radius;
}

void parallel_pixelMapping::operator()(const cv::Range& r) const{
	
	int f = getLimbmap()[limbid].first;
	int s = getLimbmap()[limbid].second;

	for(int i=r.start; i<r.end; ++i)
	{
		std::vector<cv::Scalar> blended;

		for(auto it=scoreList->begin(); it!=scoreList->end(); ++it){

			CroppedCvMat texture;
			
			if(blendMode == CMPC_NO_OCCLUSION){
				texture = (*vidRecord)[it->first].videoFrame;
			}else{
				texture = (*limbrary).frames[it->first][limbid];
			}

			cv::Vec3f _to_a = mat_to_vec3((*vidRecord)[it->first].kinectPoints.points.col(f));
			cv::Vec3f _to_b = mat_to_vec3((*vidRecord)[it->first].kinectPoints.points.col(s));

			cv::Vec3f to_a = _to_b + cylinderBody->newLeftOffset_cyl[limbid] * (_to_a - _to_b);
			cv::Vec3f to_b = _to_a + cylinderBody->newRightOffset_cyl[limbid] * (_to_b - _to_a);

			cv::Point2i pt = mapPixel
				((*fromPixels)[i], texture.offset,
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
			(*erasevector)[i] = true;
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
				pixelColors[i] = (blendPixel);
			}else{
				(*erasevector)[i] = true;
			}
		}

		
	}

}