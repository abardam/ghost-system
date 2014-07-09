#include <Windows.h>

#include "CylinderBody.h"
#include "cvutil.h"
#include "definitions.h"
#include "ghostutil.h"
#include "loader.h"
#include "KinectManager.h"
#include "ghostcam.h"
#include "tinyxml.h"
#include "texturesearch.h"

#include <array>
#include <list>
#include <cstdlib>

void CylinderBody::Reset(){
	vidRecordBins.clear();
	for(int i=0;i<NUMLIMBS;++i){
		partRadii[i] = 0;
		leftOffset[i] = 0;
		rightOffset[i] = 0;
		fixedPartRadii[i] = 0;
	}
}


void CylinderBody::setVidRecord(std::vector<SkeleVideoFrame> * vidRecord){
	this->vidRecord = vidRecord;
}

bool MyDataSortPredicate(const std::pair<int, float>& lhs, std::pair<int, float>& rhs) 
{ 
  return lhs.second < rhs.second; 
} 

int CylinderBody::getBestFrame(cv::Vec3f facing, int frameno, bool writeim){

	facing = cv::normalize(facing);

	float score;
	int currentbest = -1;

	if(!writeim)
	{

		for(int i=0; i<vidRecord->size(); ++i){

			float cscore = cv::norm( (*vidRecord)[i].facing - facing );

			//int diff = (frameno - i);

			//if(std::abs(diff) > 5) diff = 5;

			//cscore += diff * diff / 100.;

			if(currentbest == -1 || cscore < score){
				score = cscore;
				currentbest = i;
			}
		}
	}
	else{
		
		std::list<std::pair<int, float>> scoreArray;

		for(int i=0; i<vidRecord->size(); ++i){
			scoreArray.push_back(std::pair<int,float>(i, cv::norm( (*vidRecord)[i].facing - facing )));
		}

		scoreArray.sort(MyDataSortPredicate);

		currentbest = scoreArray.front().first;

		cv::imwrite("comp1.png", (*vidRecord)[currentbest].videoFrame.mat);
		scoreArray.pop_front();
		cv::imwrite("comp2.png", (*vidRecord)[scoreArray.front().first].videoFrame.mat);
		scoreArray.pop_front();
		cv::imwrite("comp3.png", (*vidRecord)[scoreArray.front().first].videoFrame.mat);
		scoreArray.pop_front();
		cv::imwrite("comp4.png", (*vidRecord)[scoreArray.front().first].videoFrame.mat);
		scoreArray.pop_front();
		cv::imwrite("comp5.png", (*vidRecord)[scoreArray.front().first].videoFrame.mat);
		scoreArray.pop_front();
		


	}

	return currentbest;

}

int CylinderBody::getBestFrame(cv::Mat s, bool writeim){
	float bestScore = -1;
	int bestFrame;
	cv::Mat b = normalizeSkeleton(s);


	if(!writeim)
	{

		for(int i=0; i<vidRecord->size(); ++i){ //NOTE: find a way to exclude invalid skeletons
		
			//if(!(*vidRecord)[i].allPartsIn) continue;
			if((*vidRecord)[i].videoFrame.mat.empty()) continue;
			
			cv::Mat a = /*normalizeSkeleton*/((*vidRecord)[i].kinectPoints2P); //should be normalized na
			float bst=calculateScore(a,b);
			/*cv::Mat tmp = b-a;
			cv::Mat tmp2;
			cv::pow(tmp, 2, tmp2);
			bst = cv::sum(tmp2)(0);*/
		
			if(bestScore == -1 || bst < bestScore){
				bestScore = bst;
				bestFrame = i;
			}
		}
	}else{
		/*
		std::list<std::pair<int, float>> scoreArray;

		for(int i=0; i<vidRecord->size(); ++i){

			if(!(*vidRecord)[i].allPartsIn) continue;
			if((*vidRecord)[i].videoFrame.mat.empty()) continue;

			float bst=0;
			cv::Mat a = ((*vidRecord)[i].kinectPoints2P);
			for(int j=0; j<NUMJOINTS; ++j){
				bst += cv::norm(b.col(j)-a.col(j));
			}

			scoreArray.push_back(std::pair<int,float>(i, bst));
		}

		scoreArray.sort(MyDataSortPredicate);

		bestFrame = scoreArray.front().first;

		cv::imwrite("comp1.png", (*vidRecord)[bestFrame].videoFrame.mat);
		scoreArray.pop_front();
		cv::imwrite("comp2.png", (*vidRecord)[scoreArray.front().first].videoFrame.mat);
		scoreArray.pop_front();
		cv::imwrite("comp3.png", (*vidRecord)[scoreArray.front().first].videoFrame.mat);
		scoreArray.pop_front();
		cv::imwrite("comp4.png", (*vidRecord)[scoreArray.front().first].videoFrame.mat);
		scoreArray.pop_front();
		cv::imwrite("comp5.png", (*vidRecord)[scoreArray.front().first].videoFrame.mat);
		scoreArray.pop_front();*/
	}


	return bestFrame;
}



void CylinderBody::getBestFrames(cv::Mat points, int arr[NUMLIMBS]){
	cv::Mat b = normalizeSkeleton(points);

	for(int part=0; part<NUMLIMBS; ++part){
		float bestScore = -1;
		int bestFrame;
		for(int i=0; i<vidRecord->size(); ++i){ 
		
			if(!(*vidRecord)[i].allPartsIn) continue;

			float bst=0;
			cv::Mat a = ((*vidRecord)[i].kinectPoints2P);

			float * weights = getPartWeights(part);

			for(int j=0; j<NUMJOINTS; ++j){
				float d = cv::norm(b.col(j)-a.col(j));
				bst += weights[j] * d;
			}

			if(bestScore == -1 || bst < bestScore){
				bestScore = bst;
				bestFrame = i;
			}
		}
		arr[part] = bestFrame;
	}
}


int CylinderBody::getBestFrameBin(cv::Mat s, int bin, bool writeim){
	float bestScore = -1;
	int bestFrame;
	cv::Mat b = normalizeSkeleton(s);


	if(!writeim)
	{

		for(int i=0; i<vidRecordBins[bin].size(); ++i){ //NOTE: find a way to exclude invalid skeletons
			SkeleVideoFrame * svf = vidRecordBins[bin][i];
			if(!(*svf).allPartsIn) continue;

			float bst=0;
			cv::Mat a = /*normalizeSkeleton*/((*svf).kinectPoints2P); //should be normalized na
			for(int j=0; j<NUMJOINTS; ++j){
				bst += cv::norm(b.col(j)-a.col(j));
			}
		
			if(bestScore == -1 || bst < bestScore){
				bestScore = bst;
				bestFrame = i;
			}
		}
	}else{



		std::list<std::pair<int, float>> scoreArray;

		for(int i=0; i<vidRecordBins[bin].size(); ++i){
			SkeleVideoFrame * svf = vidRecordBins[bin][i];
			float bst=0;
			cv::Mat a = /*normalizeSkeleton*/((*svf).kinectPoints2P);
			for(int j=0; j<NUMJOINTS; ++j){
				bst += cv::norm(b.col(j)-a.col(j));
			}

			scoreArray.push_back(std::pair<int,float>(i, bst));
		}

		scoreArray.sort(MyDataSortPredicate);

		bestFrame = scoreArray.front().first;

		cv::imwrite("comp1.png", vidRecordBins[bin][bestFrame]->videoFrame.mat);
		scoreArray.pop_front();
		cv::imwrite("comp2.png", vidRecordBins[bin][scoreArray.front().first]->videoFrame.mat);
		scoreArray.pop_front();
		cv::imwrite("comp3.png", vidRecordBins[bin][scoreArray.front().first]->videoFrame.mat);
		scoreArray.pop_front();
		cv::imwrite("comp4.png", vidRecordBins[bin][scoreArray.front().first]->videoFrame.mat);
		scoreArray.pop_front();
		cv::imwrite("comp5.png", vidRecordBins[bin][scoreArray.front().first]->videoFrame.mat);
		scoreArray.pop_front();
	}


	return bestFrame;
}
	
void CylinderBody::calcFacings(){
	for(auto it=vidRecord->begin();it != vidRecord->end(); ++it){

		it->facing = KINECT::calculateFacing(&(it->kinectPoints));

	}

	//orientation "tracking"

	if((*vidRecord)[0].facing[2] > 0) (*vidRecord)[0].facing *= -1;
	for(int i=1; i<vidRecord->size(); ++i){
		cv::Vec3f a = (*vidRecord)[i].facing;
		cv::Vec3f b = (*vidRecord)[i-1].facing;

		(*vidRecord)[i].facing *= cv::norm(a - b) < cv::norm(-a - b) ? 1 : -1;


	}

}

#define GOODJOINTTHRESHOLD 10

void CylinderBody::validateLimbs(){
	
	//check vidrecord if body parts are within frame
	for(auto it=vidRecord->begin(); it != vidRecord->end(); ++it){
		/*it->allPartsIn = true;
		for(auto it2 = it->skeleton.states.begin(); it2 != it->skeleton.states.end(); ++it2){
			if( *it2 != NUI_SKELETON_POSITION_TRACKED){
				it->allPartsIn = false;
				continue;
			}
		}*/

		int numGoodJoints = 0;

		for(int i=0;i<NUMJOINTS;++i){
			if(KINECT::checkTracked(it->kinectPoints.states[i]))
				++numGoodJoints;
		}

		if(numGoodJoints > GOODJOINTTHRESHOLD){
			it->allPartsIn = true;
		}else{
			it->allPartsIn = false;
		}
	}
}
	
IMGPIXEL CylinderBody::getColorAtPartAndPixel(int frame, int part, cv::Vec4f pixel, cv::Vec2f * pixelLoc){

	SkeleVideoFrame * targetFrame = &(*vidRecord)[frame];
	
	cv::Vec2f pix =  toScreen( mat_to_vec3( limbTransforms[part][frame] * cv::Mat(pixel) ));
	cv::Point2d offset = targetFrame->videoFrame.offset;
	pix[0] -= offset.x;
	pix[1] -= offset.y;

	if(pix[0] < 0 || pix[0] >= targetFrame->videoFrame.mat.cols || pix[1] < 0 || pix[1] >= targetFrame->videoFrame.mat.rows)
		return IMGPIXEL(0,0,0);

	IMGPIXEL col = targetFrame->videoFrame.mat.at<IMGPIXEL>( cv::Point(pix[0], pix[1]) );

	//std::cout << pix << std::endl << col << std::endl;

	if(pixelLoc != 0){
		*pixelLoc = pix;
	}

	return col;
}

cv::Mat CylinderBody::getCylinderTransform(int part, int frame){

	SkeleVideoFrame * targetFrame = &(*vidRecord)[frame];

	cv::Vec3f v3A = mat_to_vec3(targetFrame->kinectPoints.points.col(getLimbmap()[part].first));
	cv::Vec3f v3B = mat_to_vec3(targetFrame->kinectPoints.points.col(getLimbmap()[part].second));

	cv::Vec3f first = v3A; // + (v3A - v3B) * -getLeftOffset()[part];
	cv::Vec3f second = v3B; // + (v3B - v3A) * getRightOffset()[part];
	
#if GH_MODELFITTING == GH_MF_OLD
	return ::getCylinderTransform(first, second, partRadii[part], leftOffset[part], rightOffset[part]);
#elif GH_MODELFITTING == GH_MF_CYLPROJ
	return ::getCylinderTransform(first, second, newPartRadii_cyl[part], newLeftOffset_cyl[part], newRightOffset_cyl[part]);
#endif

}

void CylinderBody::calcLimbTransforms(){
	for(int i=0;i<NUMLIMBS;++i){
		int i_ = i;
		for(int f=0;f<vidRecord->size();++f)
			limbTransforms[i_][f] = getCylinderTransform(i, f);
	}
}



void CylinderBody::regularizeLimbs(){

	//partRadii[UPPERARM_LEFT]	*= 4;
	//partRadii[UPPERARM_RIGHT]	*= 4;
	//partRadii[LOWERLEG_LEFT] = partRadii[UPPERARM_LEFT];
	//partRadii[LOWERLEG_RIGHT] = partRadii[UPPERARM_LEFT];
	//partRadii[LOWERARM_LEFT] = partRadii[UPPERARM_LEFT];
	//partRadii[LOWERARM_RIGHT] = partRadii[UPPERARM_LEFT];
	//partRadii[UPPERLEG_LEFT] = partRadii[UPPERARM_LEFT];
	//partRadii[UPPERLEG_RIGHT] = partRadii[UPPERARM_LEFT];

	//model symmetry

	float srad, sleft, sright;
	int pairs[] = {UPPERARM_LEFT, UPPERARM_RIGHT, UPPERLEG_LEFT, UPPERLEG_RIGHT, LOWERARM_LEFT, LOWERARM_RIGHT, LOWERLEG_LEFT, LOWERLEG_RIGHT};
#pragma push_macro("max")
#undef max
#pragma push_macro("min")
#undef min

	for(int i=0;i<8;i+=2){
		srad = std::max(partRadii[pairs[i]], partRadii[pairs[i+1]]);
		sleft = std::min(leftOffset[pairs[i]], leftOffset[pairs[i+1]]);
		sright = std::max(rightOffset[pairs[i]], rightOffset[pairs[i+1]]);
		partRadii[pairs[i]] = srad;
		partRadii[pairs[i+1]] = srad;
		leftOffset[pairs[i]] = sleft;
		leftOffset[pairs[i+1]] = sleft;
		rightOffset[pairs[i]] = sright;
		rightOffset[pairs[i+1]] = sright;
	}

#pragma pop_macro min
#pragma pop_macro max

	//head widen

	//partRadii[HEAD] *= 2;

}


void Reset(){

}

/*void CylinderBody::colorsAtPixels(std::vector<cv::Vec4f> * pt3d, cv::Mat unrotmat2, int bestFrame, int part, std::vector<std::pair<IMGPIXEL, cv::Vec3f>> * colorPointVector){
	for(auto it=pt3d->begin(); it!=pt3d->end(); ++it){
		cv::Mat temp =  cv::Mat(*it);
		cv::Mat temp2 =  unrotmat2 * cv::Mat(*it);

		cv::Vec3f tempVec;

		tempVec = mat2Vec(temp);

		IMGPIXEL color;

		//color = getColorFromPixel(bestFrame, mat2Vec(temp2), i, &pixloc);
		//getcoloratpartnpixel

		SkeleVideoFrame * targetFrame = &(*vidRecord)[bestFrame];
	
		cv::Vec2f pix =  toScreen( mat2Vec( limbTransforms[part][bestFrame] * temp2 ));

		if(pix[0] < 0 || pix[0] >= targetFrame->videoFrame.cols || pix[1] < 0 || pix[1] >= targetFrame->videoFrame.rows)
			continue;

		color = targetFrame->videoFrame.at<IMGPIXEL>( cv::Point(pix[0], pix[1]) );

		if(color == WHITE) continue;

		colorPointVector->push_back(std::pair<IMGPIXEL, cv::Vec3f>(color, tempVec));
	}
}*/

void CylinderBody::colorsAtPixels(std::vector<cv::Vec4f> * pt3d, cv::Mat unrotmat2, int bestFrame, int part, std::vector<std::pair<IMGPIXEL, cv::Vec3f>> * colorPointVector, cv::Mat * toColorPic, bool whitePixels){
	SkeleVideoFrame * targetFrame = &(*vidRecord)[bestFrame];
	cv::Point2f offset = targetFrame->videoFrame.offset;
	cv::Mat unrotlimb =  limbTransforms[part][bestFrame] * unrotmat2;

	for(auto it=pt3d->begin(); it!=pt3d->end(); ++it){
		cv::Mat temp =  cv::Mat(*it);

		cv::Vec3f tempVec;

		tempVec = mat_to_vec3(temp);

		IMGPIXEL color;

		cv::Mat tempPix = getCameraMatrix() * unrotlimb * temp;

		//cv::Vec2f pix =  toScreen( mat2Vec(  unrotlimb * temp ));

		cv::Vec2f pix(tempPix.at<float>(0)/tempPix.at<float>(2), tempPix.at<float>(1)/tempPix.at<float>(2));

		if(pix[0] < 0 || pix[0] >= targetFrame->videoFrame.mat.cols || pix[1] < 0 || pix[1] >= targetFrame->videoFrame.mat.rows)
			continue;
		
		pix[0] -= offset.x;
		pix[1] -= offset.y;

		if(whitePixels) color = WHITE;
		else color = targetFrame->videoFrame.mat.at<IMGPIXEL>( cv::Point(pix[0], pix[1]) );

		if(toColorPic != NULL)
			toColorPic->at<IMGPIXEL>(pix(1), pix(0)) = IMGPIXEL(0xff, 0, 0, 0xff);

		if(color == WHITE && !whitePixels) continue;

		colorPointVector->push_back(std::pair<IMGPIXEL, cv::Vec3f>(color, tempVec)); //change to image
	}
}

void CylinderBody::colorsAtPixels(std::vector<VecPart> * pt3d, cv::Mat unrotmat2[NUMLIMBS], int bestFrame, cv::Mat * out, cv::Mat * toColorPic, bool whitePixels){
	//out->create(HEIGHT, WIDTH, cv::DataType<IMGPIXEL>::type);
	

	SkeleVideoFrame * targetFrame = &(*vidRecord)[bestFrame];
	cv::Point2d offset = targetFrame->videoFrame.offset;
	cv::Mat unrotlimb[NUMLIMBS];
	for(int i=0;i<getCombineParts().size();++i){
		int part = getCombineParts()[i][0];
		unrotlimb[part] =  limbTransforms[part][bestFrame] * unrotmat2[part];
	}

	for(auto it=pt3d->begin(); it!=pt3d->end(); ++it){
		cv::Mat temp =  cv::Mat(it->vec);

		cv::Vec3f tempVec;

		tempVec = mat_to_vec3(temp);

		int realpart = getCombinePartsMap()[it->part];
		if(realpart == -1) continue;

		IMGPIXEL color;	

		cv::Mat tempPix = getCameraMatrix() * unrotlimb[realpart] * temp;
		//cv::Vec2f pixo =  toScreen( mat2Vec(  unrotlimb[realpart] * temp ));
		cv::Vec2f pix(tempPix.at<float>(0)/tempPix.at<float>(2), tempPix.at<float>(1)/tempPix.at<float>(2));
		
		pix[0] -= offset.x;
		pix[1] -= offset.y;
		//std::cout << pix << "\t" << pixo << std::endl;

		if(pix[0] < 0 || pix[0] >= targetFrame->videoFrame.mat.cols || pix[1] < 0 || pix[1] >= targetFrame->videoFrame.mat.rows)
			continue;

		if(whitePixels) color = WHITE;
		else color = targetFrame->videoFrame.mat.at<IMGPIXEL>( cv::Point(pix[0], pix[1]) );

		if(toColorPic != NULL)
			toColorPic->at<IMGPIXEL>(pix(1), pix(0)) = IMGPIXEL(0xff, 0, 0);

		if(color == WHITE && !whitePixels) continue;

		cv::Point p((tempVec(0)/tempVec(2)+1)*out->cols/2, (tempVec(1)/tempVec(2)+1)*out->rows/2);

		if(p.x >= 0 && p.x < out->cols && p.y >= 0 && p.y < out->rows)
			out->at<IMGPIXEL>(p) = color;
		//colorPointVector->push_back(std::pair<IMGPIXEL, cv::Vec3f>(color, tempVec)); //change to image
	}
}

void CylinderBody::colorsAtPixels(cv::Mat pt3d[NUMLIMBS], cv::Mat unrotmat2[NUMLIMBS], int bestFrame, cv::Mat * out, int * order, cv::Mat * toColorPic, bool whitePixels, bool shittyOption){
	SkeleVideoFrame * targetFrame = &(*vidRecord)[bestFrame];
	
	cv::Point2d offset = targetFrame->videoFrame.offset;

	cv::Mat unrotlimb[NUMLIMBS];
	for(int i=0;i<getCombineParts().size();++i){
		int part = getCombineParts()[i][0];
		//revert this
		int part2;
		if(shittyOption) part2 = UPPERARM_LEFT;
		else part2 = part;
		unrotlimb[part] =  limbTransforms[part2][bestFrame] * unrotmat2[part2];
	}


	IMGPIXEL color;

	for(int i_=0;i_<NUMLIMBS;++i_){
		int i=order[i_];

		if(pt3d[i].empty()) continue;

		cv::Mat temp = getCameraMatrix() * unrotlimb[i] * pt3d[i];

		for(int j=0;j<temp.cols;++j){
			cv::Vec2f pix(temp.at<float>(0,j)/temp.at<float>(2,j),
				temp.at<float>(1,j)/temp.at<float>(2,j));

			pix[0] -= offset.x;
			pix[1] -= offset.y;

			if(pix[0] < 0 || pix[0] >= targetFrame->videoFrame.mat.cols || pix[1] < 0 || pix[1] >= targetFrame->videoFrame.mat.rows)
				continue;

			if(whitePixels) color = WHITE;
			else color = targetFrame->videoFrame.mat.at<IMGPIXEL>( cv::Point(pix[0], pix[1]) );

			if(toColorPic != NULL){
				toColorPic->at<IMGPIXEL>(pix(1), pix(0)) = IMGPIXEL(0xff, 0, 0)*0.1 + toColorPic->at<IMGPIXEL>(pix(1),pix(0))*0.9;
			}

			if(color == WHITE && !whitePixels) continue;

			cv::Point p((pt3d[i].at<float>(0,j)/pt3d[i].at<float>(2,j)+1)*out->cols/2, (pt3d[i].at<float>(1,j)/pt3d[i].at<float>(2,j)+1)*out->rows/2);

			if(p.x >= 0 && p.x < out->cols && p.y >= 0 && p.y < out->rows)
				out->at<IMGPIXEL>(p) = color;
		}
	}
}

void CylinderBody::colorsAtPixelsMultiFrame(cv::Mat pt3d[NUMLIMBS], cv::Mat unrotmat2[NUMLIMBS], int bestFrames[NUMLIMBS], cv::Mat * out){

	cv::Mat unrotlimb[NUMLIMBS];
	
	IMGPIXEL color;

	for(int i=0;i<NUMLIMBS;++i){
		if(pt3d[i].empty()) continue;

		for(int i=0;i<getCombineParts().size();++i){
			int part = getCombineParts()[i][0];
			unrotlimb[part] =  limbTransforms[part][bestFrames[i]] * unrotmat2[part];
		}

		SkeleVideoFrame * targetFrame = &(*vidRecord)[bestFrames[i]];
	
		cv::Point2d offset = targetFrame->videoFrame.offset;
		cv::Mat temp = getCameraMatrix() * unrotlimb[i] * pt3d[i];

		for(int j=0;j<temp.cols;++j){
			cv::Vec2f pix(temp.at<float>(0,j)/temp.at<float>(2,j),
				temp.at<float>(1,j)/temp.at<float>(2,j));

			pix[0] -= offset.x;
			pix[1] -= offset.y;

			if(pix[0] < 0 || pix[0] >= targetFrame->videoFrame.mat.cols || pix[1] < 0 || pix[1] >= targetFrame->videoFrame.mat.rows)
				continue;

			color = targetFrame->videoFrame.mat.at<IMGPIXEL>( cv::Point(pix[0], pix[1]) );

			if(color == WHITE) continue;

			cv::Point p((pt3d[i].at<float>(0,j)/pt3d[i].at<float>(2,j)+1)*out->cols/2, (pt3d[i].at<float>(1,j)/pt3d[i].at<float>(2,j)+1)*out->rows/2);

			if(p.x >= 0 && p.x < out->cols && p.y >= 0 && p.y < out->rows)
				out->at<IMGPIXEL>(p) = color;
		}
	}
}

//bins the vidrecord svf's
void CylinderBody::calcVidRecordBins(){
	vidRecordBins.clear();

	vidRecordBins.resize(NUMBINS);

	for(auto it=vidRecord->begin(); it != vidRecord->end(); ++it){
		int bin = calcBinFromFacing(it->facing);
		if(bin<0) bin = 0;
		if(bin>=NUMBINS) bin = NUMBINS-1;

		vidRecordBins[bin].push_back(&*it);
	}

	for(int i=0;i<NUMBINS;++i){
		std::cout << vidRecordBins[i].size() << std::endl;
	}

	//make sure no bin is empty

	for(int i=0;i<NUMBINS;++i){
		if(!vidRecordBins[i%(int)NUMBINS].empty()){
			if(vidRecordBins[(i+1)%(int)NUMBINS].empty()){
				for(auto it=vidRecordBins[i%(int)NUMBINS].begin();it!=vidRecordBins[i%(int)NUMBINS].end();++it){
					vidRecordBins[(i+1)%(int)NUMBINS].push_back(*it);
				}
			}
		}
		int j=NUMBINS-i;
		
		if(!vidRecordBins[j%(int)NUMBINS].empty()){
			if(vidRecordBins[(j-1)%(int)NUMBINS].empty()){
				for(auto it=vidRecordBins[j%(int)NUMBINS].begin();it!=vidRecordBins[j%(int)NUMBINS].end();++it){
					vidRecordBins[(j-1)%(int)NUMBINS].push_back(*it);
				}
			}
		}
	}


	//debug:
	//check binnings;

	std::string dir = "bin";

	for(int i=0;i<NUMBINS;++i){
		char a[4];
		std::string dir2 = dir + std::string(_itoa(i, a, 10));

		CreateDirectoryA(dir2.c_str(), NULL);

		std::string fname = "im";
		
		for(int j=0;j<vidRecordBins[i].size();++j){
			std::string fname2 = fname + std::string(_itoa(j,a,10)) + ".png";

			cv::imwrite(dir2 + "/" + fname2, vidRecordBins[i][j]->videoFrame.mat);
		}
	}
}

void CylinderBody::Save(std::string path){
	std::string filepath = path + "/cylinderbody.xml";

	CreateDirectoryA(path.c_str(), NULL);

	TiXmlDocument xmlDoc;
	TiXmlDeclaration * decl = new TiXmlDeclaration("1.0", "", "");
	xmlDoc.LinkEndChild(decl);

	TiXmlElement * root = new TiXmlElement("CylinderBody");
	xmlDoc.LinkEndChild(root);
	root->SetDoubleAttribute("version", 0.1);
	root->SetAttribute("bestframe", bestFrame);
	root->SetAttribute("numlimbs", NUMLIMBS);

	for(int i=0; i<NUMLIMBS; ++i){
		TiXmlElement * limb = new TiXmlElement("LimbDefinition");
		root->LinkEndChild(limb);
		
		limb->SetAttribute("limbID", i);
		
		limb->SetDoubleAttribute("partRadius", partRadii[i]);
		limb->SetDoubleAttribute("leftOffset", leftOffset[i]);
		limb->SetDoubleAttribute("rightOffset", rightOffset[i]);
		
		limb->SetDoubleAttribute("fixedPartRadius", fixedPartRadii[i]);

		limb->SetDoubleAttribute("varianceX", varianceX[i]);
		limb->SetDoubleAttribute("varianceY", varianceY[i]);
		
		limb->SetDoubleAttribute("newPartRadius", newPartRadii[i]);
		limb->SetDoubleAttribute("newLeftOffset", newLeftOffset[i]);
		limb->SetDoubleAttribute("newRightOffset", newRightOffset[i]);

		limb->SetDoubleAttribute("newPartRadiusCyl", newPartRadii_cyl[i]);
		limb->SetDoubleAttribute("newLeftOffsetCyl", newLeftOffset_cyl[i]);
		limb->SetDoubleAttribute("newRightOffsetCyl", newRightOffset_cyl[i]);
	}

	xmlDoc.SaveFile(filepath);
}

void CylinderBody::Load(std::string path){
	std::string filepath = path + "/cylinderbody.xml";
	TiXmlDocument xmlDoc;

	if(!xmlDoc.LoadFile(filepath)){
		std::cerr << "Could not read " << filepath << "; load failed\n";
		return;
	}

	TiXmlHandle doc(&xmlDoc);
	TiXmlElement * root = doc.FirstChildElement().ToElement();

	if(!root){
		std::cerr << "No root handle; load failed\n";
		return;
	}

	double version;
	int numlimbs;
	int bestFrame;

	root->QueryDoubleAttribute("version", &version);
	root->QueryIntAttribute("numlimbs", &numlimbs);
	if(root->QueryIntAttribute("bestframe", &bestFrame) == TIXML_SUCCESS){
		this->bestFrame = bestFrame;
	}else this->bestFrame = 0;

	TiXmlHandle rootHandle(root);

	int i=0;
	for(TiXmlElement * limb = rootHandle.FirstChild().ToElement();
		limb != NULL; limb = limb->NextSiblingElement()){
			limb->QueryIntAttribute("limbID", &i);

			limb->QueryFloatAttribute("partRadius", &partRadii[i]);
			limb->QueryFloatAttribute("leftOffset", &leftOffset[i]);
			limb->QueryFloatAttribute("rightOffset", &rightOffset[i]);
			
			limb->QueryFloatAttribute("fixedPartRadius", &fixedPartRadii[i]);
			
			limb->QueryFloatAttribute("varianceX", &varianceX[i]);
			limb->QueryFloatAttribute("varianceY", &varianceY[i]);
			
			limb->QueryFloatAttribute("newPartRadius", &newPartRadii[i]);
			limb->QueryFloatAttribute("newLeftOffset", &newLeftOffset[i]);
			limb->QueryFloatAttribute("newRightOffset", &newRightOffset[i]);
			
			limb->QueryFloatAttribute("newPartRadiusCyl", &newPartRadii_cyl[i]);
			limb->QueryFloatAttribute("newLeftOffsetCyl", &newLeftOffset_cyl[i]);
			limb->QueryFloatAttribute("newRightOffsetCyl", &newRightOffset_cyl[i]);

	}


}