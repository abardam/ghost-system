#include "ghostmapping.h"
#include "bodybuild.h"
#include "cvutil.h"
#include "ghostcam.h"
#include <omp.h>

//remove later
//used for facingHelper
#include "KinectManager.h"

#if 0
//old function
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

	size_t fromsize = fromPixels_2d_v.size();
	std::vector<bool> erasevector(fromsize, false);
	std::vector<cv::Scalar> pixelColors(fromsize);


	//concurrency::parallel_for(size_t(0), fromsize, [&](size_t i)
	for(int i=0;i<fromsize;++i)
	{
		std::vector<cv::Scalar> blended;

		for(auto it=scoreList.begin(); it!=scoreList.end(); ++it){

			CroppedCvMat texture;
			
			if(blendMode == CMPC_NO_OCCLUSION){
				texture = (*vidRecord)[it->first].videoFrame;
			}else{
				texture = (*limbrary).frames[it->first][limbid];
			}

			//cv::Vec3f _to_a = mat_to_vec3((*vidRecord)[it->first].kinectPoints.points.col(f));
			//cv::Vec3f _to_b = mat_to_vec3((*vidRecord)[it->first].kinectPoints.points.col(s));
			//
			//cv::Vec3f to_a = _to_b + cylinderBody->newLeftOffset_cyl[limbid] * (_to_a - _to_b);
			//cv::Vec3f to_b = _to_a + cylinderBody->newRightOffset_cyl[limbid] * (_to_b - _to_a);

			if(!(*vidRecord)[it->first].kinectPoints.offsetPointsCalculated){
				std::cerr << "error: offset points not calculated!\n";
				throw;
			}

			cv::Vec3f to_a = mat_to_vec3((*vidRecord)[it->first].kinectPoints.offsetPoints.col(limbid+0));
			cv::Vec3f to_b = mat_to_vec3((*vidRecord)[it->first].kinectPoints.offsetPoints.col(limbid+1));

			cv::Point2i pt = mapPixel
				(fromPixels[i], texture.offset,
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
			erasevector[i] = true;
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
				erasevector[i] = true;
			}
		}

		
	}

	auto fromptr = fromPixels_2d_v.begin();
	auto pixptr = pixelColors.begin();
	for(int i=0;i<fromsize;++i){
		if(erasevector[i]){
			fromptr = fromPixels_2d_v.erase(fromptr);
			pixptr = pixelColors.erase(pixptr);
		}else{
			++fromptr;
			++pixptr;
		}
	}

	return PixelColorMap(fromPixels_2d_v, pixelColors);
}
#endif

#if 0

//TODO: fix this shitty argument list
void cylinderMapPixelsColor_parallel(	cv::Vec3f from_a[NUMLIMBS], 
										cv::Vec3f from_b[NUMLIMBS], 
										float facing[NUMLIMBS], 
										ScoreList scoreList[NUMLIMBS],
										cv::Point voff[NUMLIMBS], 
										std::vector<SkeleVideoFrame> * vidRecord, 
										CylinderBody * cylinderBody, 
										Limbrary * limbrary, 
										int blendMode,
										cv::Mat fromPixels[NUMLIMBS],
										std::vector<cv::Vec3s> fromPixels_2d_v[NUMLIMBS],
										//PixelColorMap from_color[NUMLIMBS])
										cv::Mat draw)
{
	
	unsigned int bl;
	switch(blendMode){
	case CMPC_NO_OCCLUSION:
		bl = 1;
		break;
	case CMPC_BLEND_NONE:
		bl = 1;
		break;
	case CMPC_BLEND_1:
		bl = 5;
		break;
	}
	const lmap const * limbmap = getLimbmap();

	const unsigned int blendLimit = bl;

	cv::Mat zBuf(HEIGHT, WIDTH, CV_16U, cv::Scalar(MAXDEPTH));

	std::vector<std::vector<cv::Mat>> candidateTextureTransformMatrices(NUMLIMBS);
	for(int i=0;i<NUMLIMBS;++i){

		for(auto it=scoreList[i].begin(); it!=scoreList[i].end(); ++it){
			
			int f = limbmap[i].first;
			int s = limbmap[i].second;

			cv::Vec3f _to_a = mat_to_vec((*vidRecord)[it->first].kinectPoints.points.col(f));
			cv::Vec3f _to_b = mat_to_vec((*vidRecord)[it->first].kinectPoints.points.col(s));

			cv::Vec3f to_a = _to_b + cylinderBody->newLeftOffset_cyl[i] * (_to_a - _to_b);
			cv::Vec3f to_b = _to_a + cylinderBody->newRightOffset_cyl[i] * (_to_b - _to_a);

			candidateTextureTransformMatrices[i].push_back(getCameraMatrix() * cylinderFacingTransform(from_a[i], from_b[i], tempCalcFacing(i, (*vidRecord)[it->first].kinectPoints), to_a, to_b, facing[i]));
		}
	}

	

	for(int limbid=0;limbid<NUMLIMBS;++limbid)
	{
	
		int f = limbmap[limbid].first;
		int s = limbmap[limbid].second;

		int i;
		int fromsize = fromPixels[limbid].size().width;

		std::vector<cv::Mat> transformedPoints;
		int scoreCand = -1;
		for(ScoreList::iterator it=scoreList[limbid].begin(); it!=scoreList[limbid].end(); ++it){
			++scoreCand;
			transformedPoints.push_back(candidateTextureTransformMatrices[limbid][scoreCand] * (fromPixels[limbid]));
		}

		for(i=0;i<fromsize;++i){

			cv::Scalar blendPixel(0,0,0);
			cv::Scalar lastBlend(0,0,0);
			float totalAlpha = 0;
			float colorAlpha = 0;
			float div = 1;
			unsigned int blends = 0;

			int scoreCand = -1;

			for(ScoreList::iterator it=scoreList[limbid].begin(); it!=scoreList[limbid].end(); ++it){

				++scoreCand;

				CroppedCvMat texture = blendMode==CMPC_NO_OCCLUSION? (*vidRecord)[it->first].videoFrame : (limbrary)->getFrameLimbs(it->first)[limbid];

				cv::imshow("texture", texture.mat);
			
				cv::Point2i pt = mat4_to_vec2(transformedPoints[limbid].col(i));
				pt -= texture.offset;

				cv::Scalar pixelColor;

				//int res = colorPixel(pt, limbid, texture, &pixelColor); //expanded

				int res;

				if(CLAMP_SIZE(pt.x, pt.y, texture.mat.cols, texture.mat.rows)){

					const cv::Vec3b& ptv = texture.mat.ptr<cv::Vec3b>(pt.y)[pt.x];
					pixelColor = cv::Scalar(ptv);
		
					if(ptv != BLUE){
						if(ptv != WHITE){

							res = CP_GOOD;
						}else{
							res = CP_BG;
						}
					}else{
						res = CP_OCCLUDED;
					}
				}else{
					res = CP_OVER;
				}
			
				if(res == CP_BG){
					//for(int j=0;j<3;++j) lastBlend(j) = 255;
					++blends;
					div/=2;
					//blendPixel += lastBlend * div;
					//totalAlpha += div;
				}else if(res == CP_GOOD || (blendMode == CMPC_NO_OCCLUSION && res != CP_OVER)){
					lastBlend = (pixelColor);
					++blends;
					div/=2;
					blendPixel += lastBlend * div;
					totalAlpha += div;
					colorAlpha += div;
				} 

				if(blends >= blendLimit) break;
			}
		
			if(blends == 0){
			}else{

				blendPixel *= 1.f/totalAlpha;

				//blend threshold; if alpha adds up to 0.5 or less, does not render the pixel
				if(colorAlpha > 0.5){
					//pixelColors[i] = (blendPixel);

					//immediately draw

					cv::Point pt(fromPixels_2d_v[limbid][i](0), fromPixels_2d_v[limbid][i](1));
					pt += voff[limbid];
				
					if(CLAMP_SIZE(pt.x,pt.y,WIDTH,HEIGHT)){

						if(fromPixels_2d_v[limbid][i](2) < zBuf.ptr<unsigned short>(pt.y)[pt.x] && fromPixels_2d_v[limbid][i](2) > 0){
							cv::Vec3b& ptColor = draw.ptr<cv::Vec3b>(pt.y)[pt.x];
							for(int k=0;k<3;++k){
								ptColor(k) = blendPixel(k);
							}

							cv::imshow("temp", draw);
							cv::waitKey(10);
			
							zBuf.ptr<unsigned short>(pt.y)[pt.x] = fromPixels_2d_v[limbid][i](2);
						}
					}
				}
			}

		
		}
	}

}


#else if 1
//TODO: fix this shitty argument list
void cylinderMapPixelsColor_parallel_orig(
		cv::Vec3f from_a[NUMLIMBS], 
		cv::Vec3f from_b[NUMLIMBS], 
		float facing[NUMLIMBS], 
		ScoreList scoreListRaw[NUMLIMBS],
		cv::Point voff[NUMLIMBS], 
		std::vector<SkeleVideoFrame> * vidRecord, 
		CylinderBody * cylinderBody, 
		Limbrary * limbrary, 
		int blendMode,
		cv::Mat fromPixels[NUMLIMBS],
		std::vector<cv::Vec3s>& fromPixels_2d_v,
		int limits[NUMLIMBS],
		//PixelColorMap from_color[NUMLIMBS])
		cv::Mat& draw,
		cv::Mat& zBuf)
{
	
	//check draw format
	if(draw.channels() != 4){
		std::cerr << "draw channels is not 4!\n";
		throw;
	}

	unsigned int imgWidth = draw.cols;
	unsigned int imgHeight = draw.rows;

	//convert from list to vector
	std::vector<int> scoreList[NUMLIMBS];
	for(int i=0;i<NUMLIMBS;++i){
		scoreList[i].reserve(scoreListRaw[i].size());
		for(auto it=scoreListRaw[i].begin(); it!=scoreListRaw[i].end(); ++it){
			scoreList[i].push_back(it->first);
		}
	}




	unsigned int bl;
	switch(blendMode){
	case CMPC_NO_OCCLUSION:
		bl = 1;
		break;
	case CMPC_BLEND_NONE:
		bl = 1;
		break;
	case CMPC_BLEND_1:
		bl = 2;
		break;
	}

	const unsigned int blendLimit = bl;
	const int fromsize = fromPixels_2d_v.size();

	const lmap const * limbmap = getLimbmap();
	
	std::vector<bool> erasevector(fromsize, false);
	std::vector<cv::Scalar> pixelColors(fromsize);

	zBuf.create(imgHeight, imgWidth, CV_16U);
	for(int i=0;i<imgHeight*imgWidth;++i){
		zBuf.ptr<unsigned short>()[i] = MAXDEPTH;
	}

	std::vector<cv::Mat> transformedPixels[NUMLIMBS];
	//std::vector<std::vector<cv::Mat>> candidateTextureTransformMatrices(NUMLIMBS);

	float cylRatio = 1.f/cylinderBody->radiusModifier;

	for(int i=0;i<NUMLIMBS;++i){
		transformedPixels[i].reserve(scoreList[i].size());
		for(auto it=scoreList[i].begin(); it!=scoreList[i].end(); ++it){
			
			//int f = limbmap[i].first;
			//int s = limbmap[i].second;

			//cv::Mat _to_a = ((*vidRecord)[*it].kinectPoints.points.col(f));
			//cv::Mat _to_b = ((*vidRecord)[*it].kinectPoints.points.col(s));
			//cv::Vec3f to_a = mat_to_vec3(_to_b + cylinderBody->newLeftOffset_cyl[i] * (_to_a - _to_b) );
			//cv::Vec3f to_b = mat_to_vec3(_to_a + cylinderBody->newRightOffset_cyl[i] * (_to_b - _to_a));

			
			if(!(*vidRecord)[*it].kinectPoints.offsetPointsCalculated){
				std::cerr << "error: offset points not calculated!\n";
				throw;
			}

			cv::Vec3f to_a = mat_to_vec3((*vidRecord)[*it].kinectPoints.offsetPoints.col(i*2+0));
			cv::Vec3f to_b = mat_to_vec3((*vidRecord)[*it].kinectPoints.offsetPoints.col(i*2+1));

			//candidateTextureTransformMatrices[i].push_back(getCameraMatrix() * cylinderFacingTransform(from_a[i], from_b[i], tempCalcFacing(i, (*vidRecord)[it->first].kinectPoints), to_a, to_b, facing[i]));

			cv::Mat transformedPixelsMat = cylinderFacingTransform(from_a[i], from_b[i], tempCalcFacing(i, (*vidRecord)[*it].kinectPoints), to_a, to_b, facing[i], cylRatio) * fromPixels[i];

#if  GHOST_CAPTURE == CAPTURE_OPENNI
			transformedPixelsMat = getCameraMatrix() * transformedPixelsMat;
			//predivide
			cv::divide(transformedPixelsMat.row(0), transformedPixelsMat.row(2), transformedPixelsMat.row(0));
			cv::divide(transformedPixelsMat.row(1), transformedPixelsMat.row(2), transformedPixelsMat.row(1));
#elif GHOST_CAPTURE == CAPTURE_KINECT2

			transformedPixelsMat = KINECT::mapCameraPointsToColorPoints(transformedPixelsMat);
#endif

			transformedPixels[i].push_back(transformedPixelsMat);
		}
	}

	//SUPER PARALLEL:
	//int transformedPixelsWidth=0;
	//int transformedPixelsLimits[NUMLIMBS];
	//for(int i=0;i<NUMLIMBS;++i){
	//	transformedPixelsWidth+=scoreList[i].size()*fromPixels[i].cols;
	//	transformedPixelsLimits[i] = transformedPixelsWidth;
	//}
	//
	//cv::Mat transformedPixels_3(3, transformedPixelsWidth, CV_32F);
	//int pixid = 0;
	//for(int i=0;i<NUMLIMBS;++i){
	//	int scoreCand = -1;
	//	for(auto it=scoreList[i].begin(); it!=scoreList[i].end(); ++it){
	//		++scoreCand;
	//		for(int j=0;j<fromPixels[i].cols;++j){
	//			transformedPixels_3.ptr<float>(0)[pixid] = transformedPixels[i][scoreCand].ptr<float>(0)[j];
	//			transformedPixels_3.ptr<float>(1)[pixid] = transformedPixels[i][scoreCand].ptr<float>(1)[j];
	//			transformedPixels_3.ptr<float>(2)[pixid] = transformedPixels[i][scoreCand].ptr<float>(2)[j];
	//			++pixid;
	//		}
	//	}
	//}
	//	
	//
	//cv::Mat transformedPixels_2(2, transformedPixelsWidth, cv::DataType<unsigned short>::type);
	//
	//int limbid;
	//#pragma omp parallel private(limbid)
	//{
	//	limbid = 0;
	//#pragma omp for
	//	for(int i=0;i<transformedPixelsWidth;++i){
	//	
	//		//while(transformedPixelsLimits[limbid]<=i)
	//		//{
	//		//	++limbid;
	//		//}
	//		//int pixid = i-(limbid==0?0:transformedPixelsLimits[limbid-1]);
	//		//int pixid2 = pixid%fromPixels[limbid].cols;
	//		//int scoreid = pixid/fromPixels[limbid].cols;
	//		//float z = transformedPixels[limbid][scoreid].ptr<float>(2)[pixid2];
	//		//unsigned short x = transformedPixels[limbid][scoreid].ptr<float>(0)[pixid2]/z;
	//		//unsigned short y = transformedPixels[limbid][scoreid].ptr<float>(1)[pixid2]/z;
	//
	//		float z = transformedPixels_3.ptr<float>(2)[i];
	//		float x = transformedPixels_3.ptr<float>(0)[i];
	//		float y = transformedPixels_3.ptr<float>(1)[i];
	//
	//		transformedPixels_2.ptr<unsigned short>(0)[i] = x/z;
	//		transformedPixels_2.ptr<unsigned short>(1)[i] = y/z;
	//	}
	//}

	int limbid=0;
	int f, s, lastlimit, lastlimit2;
	
	f = limbmap[limbid].first;
	s = limbmap[limbid].second;
	lastlimit = 0;
	lastlimit2 = 0;

	int i;
	int nThreads;
	std::vector<cv::Mat> colorMatrices;
	std::vector<cv::Mat> depthMatrices;


//#pragma omp parallel private(i, limbid, f, s) shared(nThreads, fromsize, colorMatrices, depthMatrices, transformedPixels)
	{
#pragma omp master
		{
		nThreads = omp_get_num_threads();
		colorMatrices.resize(nThreads);
		depthMatrices.resize(nThreads);
		}
#pragma omp barrier

		int tnum = omp_get_thread_num();
		colorMatrices[tnum] = cv::Mat(imgHeight, imgWidth,CV_8UC4,cv::Scalar(255,255,255,0));
		depthMatrices[tnum] = cv::Mat(imgHeight, imgWidth, CV_16U, cv::Scalar(MAXDEPTH));
		
		limbid = 0;


#pragma omp for
	for(i=0;i<fromsize;++i){


		//for(int j=0;j<NUMLIMBS;++j){
		//	if((j==0 && i<limits[j])
		//		||(i>=limits[j-1] && i < limits[j]))
		//	{
		//		limbid = j;
		//		f = limbmap[limbid].first;
		//		s = limbmap[limbid].second;
		//		break;
		//	}
		//}
		while(limits[limbid]<=i)
		{
			++limbid;
			f = limbmap[limbid].first;
			s = limbmap[limbid].second;
			lastlimit = limits[limbid-1];
			//lastlimit2 = transformedPixelsLimits[limbid-1];
		}
		
		cv::Vec3b blendPixel(0,0,0);
		cv::Vec3b lastBlend(0,0,0);
		float totalAlpha = 0;
		float colorAlpha = 0;
		float div = 1;
		unsigned int blends = 0;

		for(int scoreCand=0;scoreCand<scoreList[limbid].size();++scoreCand){

			CroppedCvMat texture = blendMode==CMPC_NO_OCCLUSION? (*vidRecord)[scoreList[limbid][scoreCand]].videoFrame : (limbrary)->frames[scoreList[limbid][scoreCand]][limbid];

			//cv::Point2i pt = mat4_to_vec2(candidateTextureTransformMatrices[limbid][scoreCand] * vec3_to_mat4(fromPixels[i]));

			int ind = i-(limbid==0?0:limits[limbid-1]);

			//cv::Point2i pt_tex = mat4_to_vec2(transformedPixels[limbid][scoreCand].col(ind)); //shitty compensation
			cv::Point2i pt_tex(transformedPixels[limbid][scoreCand].ptr<float>(0)[ind], transformedPixels[limbid][scoreCand].ptr<float>(1)[ind]); //we predivided it up there
			pt_tex -= texture.offset;

			//int c = i-lastlimit+scoreCand*fromPixels[limbid].cols+lastlimit2;
			//unsigned short x = transformedPixels_2.ptr<unsigned short>(0)[c];
			//unsigned short y = transformedPixels_2.ptr<unsigned short>(1)[c];
			//cv::Point2i pt_tex(x,y);
			//pt_tex -= texture.offset;

			//int res = colorPixel(pt, limbid, texture, &pixelColor); //expanded

			int res;

			if(CLAMP_SIZE(pt_tex.x, pt_tex.y, texture.mat.cols, texture.mat.rows)){

				const cv::Vec3b& ptv = texture.mat.ptr<cv::Vec3b>(pt_tex.y)[pt_tex.x];
				lastBlend = ptv;
		
				if(ptv != BLUE){
					if(ptv != WHITE){

						res = CP_GOOD;
					}else{
						res = CP_BG;
					}
				}else{
					res = CP_OCCLUDED;
				}
			}else{
				res = CP_OVER;
			}
			
			if(res == CP_BG){
				//for(int j=0;j<3;++j) lastBlend(j) = 255;
				++blends;
				div/=2;
				//blendPixel += lastBlend * div;
				//totalAlpha += div;
			}else if(res == CP_GOOD || (blendMode == CMPC_NO_OCCLUSION && res != CP_OVER)){
				++blends;
				div/=2;
				blendPixel += lastBlend * div;
				totalAlpha += div;
				colorAlpha += div;
			} 

			if(blends >= blendLimit) break;
		}

		if(blends == 0){
			erasevector[i] = true;
		}else{

			blendPixel *= 1.f/totalAlpha;

			//blend threshold; if alpha adds up to 0.5 or less, does not render the pixel
			if(colorAlpha > 0.5){
				//pixelColors[i] = (blendPixel);

				//immediately draw

					cv::Point pt(fromPixels_2d_v[i](0), fromPixels_2d_v[i](1));
					pt += voff[limbid];
					
#if 0
					if(fromPixels_2d_v[i](2) < zBuf.ptr<unsigned short>(pt.y)[pt.x] && fromPixels_2d_v[i](2) > 0){

//#pragma omp critical
						{
							if(fromPixels_2d_v[i](2) < zBuf.ptr<unsigned short>(pt.y)[pt.x] && fromPixels_2d_v[i](2) > 0){
								cv::Vec3b& ptColor = draw.ptr<cv::Vec3b>(pt.y)[pt.x];
								ptColor = blendPixel;
			
								zBuf.ptr<unsigned short>(pt.y)[pt.x] = fromPixels_2d_v[i](2);
							}
						}
					}
#endif
					if(CLAMP_SIZE(pt.x, pt.y, imgWidth, imgHeight)){
						if(fromPixels_2d_v[i](2) < depthMatrices[tnum].ptr<unsigned short>(pt.y)[pt.x] && fromPixels_2d_v[i](2) > 0){
						int tnum = omp_get_thread_num();
							cv::Vec4b& ptColor = colorMatrices[tnum].ptr<cv::Vec4b>(pt.y)[pt.x];
							for(int k=0;k<3;++k){
								ptColor(k) = blendPixel(k);
							}
							ptColor(3) = 255;
			
							depthMatrices[tnum].ptr<unsigned short>(pt.y)[pt.x] = fromPixels_2d_v[i](2);
						}
					}

			}else{
				erasevector[i] = true;
			}
		}

		
	}


#pragma omp barrier
#pragma omp for
	for(int i=0;i<imgWidth*imgHeight;++i){
		for(int j=0;j<nThreads;++j){
			unsigned short& zvalue = zBuf.ptr<unsigned short>()[i];
			if(zvalue > depthMatrices[j].ptr<unsigned short>()[i]){
				zvalue = depthMatrices[j].ptr<unsigned short>()[i];
				draw.ptr<cv::Vec4b>()[i] = colorMatrices[j].ptr<cv::Vec4b>()[i];
			}
		}
	}

	}
}

#endif

int colorPixel(const cv::Point2i& pt, const int& limbid, const CroppedCvMat& texture, cv::Scalar * const pixelColor){


	if(CLAMP_SIZE(pt.x, pt.y, texture.mat.cols, texture.mat.rows)){

		const cv::Vec3b& ptv = texture.mat.ptr<cv::Vec3b>(pt.y)[pt.x];

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
		//*pixelColor = (cv::Scalar(texture.mat.at<cv::Vec3b>(pt)));
		//
		//if(texture.mat.at<cv::Vec3b>(pt) != BLUE){
		//	if(texture.mat.at<cv::Vec3b>(pt) != WHITE){
		*pixelColor = cv::Scalar(ptv);
		
		if(ptv != BLUE){
			if(ptv != WHITE){
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


float tempCalcFacing(int limb, Skeleton s){
	if(limb == HEAD || limb == CHEST || limb == ABS){
		std::pair<int,int> p = KINECT::facingHelper(limb==ABS?2:1);
		cv::Mat a = s.points.col(p.first);
		cv::Mat b = s.points.col(p.second);

		cv::Vec3f perp = mat_to_vec3(b-a);

		cv::Vec3f a2 = mat_to_vec3(s.points.col(getLimbmap()[limb].first));
		cv::Vec3f b2 = mat_to_vec3(s.points.col(getLimbmap()[limb].second));

		cv::Vec3f axis_facing = cylinderFacingVector(a2,b2,0);

		cv::Vec3f true_facing = (b2-a2).cross(perp);

		return axis_facing.dot(true_facing);

	}else
		return 0;
}
