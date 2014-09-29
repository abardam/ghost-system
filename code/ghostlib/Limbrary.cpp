#include <Windows.h>
#include <iomanip>

#include "Limbrary.h"
#include "CylinderBody.h"
#include "ghostcam.h"
#include "cvutil.h"
#include "bodybuild.h"
#include "cylinderintersection.h"
#include "cylinderprojection.h"
#include "tinyxml.h"
#include "texturesearch.h"
#include "ghostutil.h"

#define OCCL_NONE 0
#define OCCL_UNOCCLUDED 1
#define OCCL_OCCLUDED 2

Limbrary::Limbrary():framesForLimb(NUMLIMBS){	
}

#define DIFFERENCE_THRESHOLD 0.1f

void Limbrary::build(std::vector<SkeleVideoFrame> * vidRecord, CylinderBody * cylinderBody, bool verbose, int startFrame, int endFrame){

	if(verbose) std::cout << "Building limbrary...\n";
	if(startFrame == -1) startFrame = 0;
	if(endFrame == -1) endFrame = vidRecord->size();

	frames.resize(vidRecord->size());

	for(int i=0;i<vidRecord->size();++i){
		frames[i] = FrameLimbs(NUMLIMBS);
	}
	
	for(int frame = startFrame; frame < endFrame; ++frame){
		//const CroppedCvMat& iVideoFrame = it->videoFrame;
		const cv::Mat iFullVideoFrame = uncrop((*vidRecord)[frame].videoFrame);
		cv::Mat iDebugDraw = iFullVideoFrame.clone();

		//cv::Mat zBuffer(iFullVideoFrame.rows, iFullVideoFrame.cols, cv::DataType<unsigned short>::type, cv::Scalar(8000));

		//cv::Mat partBuffer(iFullVideoFrame.rows, iFullVideoFrame.cols, CV_8U, cv::Scalar(NUMLIMBS));
		CroppedCvMat occlBuffer[NUMLIMBS]; 
		FrameLimbs individualLimbMat(NUMLIMBS);

		if ((*vidRecord)[frame].videoFrame.origWidth == 0 
			|| (*vidRecord)[frame].videoFrame.origHeight == 0
			|| (*vidRecord)[frame].videoFrame.mat.empty()) continue;

		bool valid[NUMLIMBS];

		for (int i = 0; i < NUMLIMBS; ++i){
			valid[i] = true;
		}

		for (int i = 0; i < NUMLIMBS; ++i){

			int f = getLimbmap()[i].first;
			int s = getLimbmap()[i].second;

			cv::Vec3f _a = mat_to_vec3((*vidRecord)[frame].kinectPoints.points.col(f));
			cv::Vec3f _b = mat_to_vec3((*vidRecord)[frame].kinectPoints.points.col(s));


			if (frame>0){
				int prevFrame = frame - 1;

				cv::Vec3f prev_a = mat_to_vec3((*vidRecord)[prevFrame].kinectPoints.points.col(f));
				cv::Vec3f prev_b = mat_to_vec3((*vidRecord)[prevFrame].kinectPoints.points.col(s));

				float difference = cv::norm(prev_a - _a) + (cv::norm(prev_b - _b));
				if (difference > DIFFERENCE_THRESHOLD){

					for (int limbid = 0; limbid < NUMLIMBS; ++limbid){

						float * weights = getPartWeights(limbid);

						if (weights[i]>0){
							valid[limbid] = false;
						}
					}

				}
			}
		}

		CroppedCvMat zBuffersLocal[NUMLIMBS];

		for(int i=0;i<NUMLIMBS;++i){

			if (!valid[i]) continue;

			int f = getLimbmap()[i].first;
			int s = getLimbmap()[i].second;

			cv::Vec3f _a = mat_to_vec3((*vidRecord)[frame].kinectPoints.points.col(f));
			cv::Vec3f _b = mat_to_vec3((*vidRecord)[frame].kinectPoints.points.col(s));

			cv::Vec3f a = _b + cylinderBody->newLeftOffset_cyl[i] * (_a-_b);
			cv::Vec3f b = _a + cylinderBody->newRightOffset_cyl[i] * (_b-_a);

			
			std::vector<cv::Vec3f> fp;
			std::vector<cv::Vec3s> fpv;
			PixelPolygon p;
			cv::Mat cylPts = cylinder_to_pts((*vidRecord)[frame].videoFrame.origWidth,
				(*vidRecord)[frame].videoFrame.origHeight, a, b, cylinderBody->newPartRadii_cyl[i], cv::Point(0,0), &p, &fp, &fpv, getCameraMatrixTexture());

			int limbpicWidth = p.hi.size()+25;

			//***DEBUG***
			//cv::Mat projectedCylPts = getCameraMatrixTexture() * cylPts;
			//for(int j=0;j<cylPts.cols;++j){
			//	cv::Point pt2d(projectedCylPts.ptr<float>(0)[j]/projectedCylPts.ptr<float>(2)[j],
			//		projectedCylPts.ptr<float>(1)[j]/projectedCylPts.ptr<float>(2)[j]);
			//
			//	if(CLAMP_SIZE(pt2d.x,pt2d.y,iDebugDraw.cols,iDebugDraw.rows))
			//		iDebugDraw.ptr<cv::Vec3b>(pt2d.y)[pt2d.x] = cv::Vec3b(200,50,50);
			//}
			//
			//cv::imshow("debug", iDebugDraw);
			//cv::waitKey();

			//cv::Point2i offset = iVideoFrame.offset + cv::Point(p.x_offset, p.lo_y);
			cv::Point2i offset = cv::Point(p.x_offset, p.lo_y);
			//cv::Mat zBufferLocal = pts_to_zBuffer(fpv, offset - voff, limbpicWidth + 1, p.hi_y - p.lo_y + 1);
			//cv::Mat zBufferLocal = pts_to_zBuffer(cylPts, iVideoFrame.offset, offset, limbpicWidth + 1, p.hi_y - p.lo_y + 1);
			cv::Mat zBufferLocal = pts_to_zBuffer(cylPts, cv::Point(0,0), offset, limbpicWidth + 1, p.hi_y - p.lo_y + 1, getCameraMatrixTexture());
			zBuffersLocal[i].mat = zBufferLocal;
			zBuffersLocal[i].offset = offset;
		}

		for(int i=0;i<NUMLIMBS;++i){
			
			individualLimbMat[i].mat = cv::Mat(zBuffersLocal[i].mat.rows, zBuffersLocal[i].mat.cols, CV_8UC3, cv::Scalar(255,255,255));
			individualLimbMat[i].offset = zBuffersLocal[i].offset;

			occlBuffer[i].mat = cv::Mat(zBuffersLocal[i].mat.rows, zBuffersLocal[i].mat.cols, CV_8U, cv::Scalar(OCCL_NONE)); //0 = no pixel, 1 = pixel unoccluded, 2 = pixel occluded
			occlBuffer[i].offset = zBuffersLocal[i].offset;

			for(int x=0; x<zBuffersLocal[i].mat.cols; ++x){
				for(int y=0; y<zBuffersLocal[i].mat.rows; ++y){

					cv::Point2i zBufferLocalPixLoc(x,y);
					cv::Point2i zBufferPixLoc = zBufferLocalPixLoc + zBuffersLocal[i].offset;

					unsigned short newDepth = zBuffersLocal[i].mat.at<unsigned short>(zBufferLocalPixLoc); //zBufferLocal.ptr<unsigned short>(zBufferLocalPixLoc.y)[zBufferLocalPixLoc.x]; //zBufferLocal.at<unsigned short>(zBufferLocalPixLoc);

					if(newDepth == MAXDEPTH) continue;

					occlBuffer[i].mat.at<unsigned char>(zBufferLocalPixLoc) = OCCL_UNOCCLUDED;

					for(int j = 0; j < NUMLIMBS; ++j){

						if(!getLimbOccludes(j, i) || j==i) continue;

						cv::Point2i zBufferOldPixLoc = zBufferPixLoc - zBuffersLocal[j].offset;

						if(CLAMP_SIZE(zBufferOldPixLoc.x, zBufferOldPixLoc.y, occlBuffer[j].mat.cols, occlBuffer[j].mat.rows)){
							unsigned short oldDepth = zBuffersLocal[j].mat.at<unsigned short>(zBufferOldPixLoc);

							if(oldDepth < newDepth){
								occlBuffer[i].mat.at<unsigned char>(zBufferLocalPixLoc) = OCCL_OCCLUDED;
							}
						}

					}
				}

			}

			//for(auto it=pts.begin(); it!=pts.end(); ++it){
			//	cv::Point a = toScreen(it->first);
			//	cv::Point b = toScreen(it->second);
			//	a -= voff;
			//	b -= voff;
			//	line(draw, a, b, cv::Scalar(255,0,0));
			//
			//}

			//cv::Mat occlBufferColor(occlBuffer[i].mat.rows,
			//	occlBuffer[i].mat.cols, CV_8UC3);
			//
			//for (int x = 0; x < occlBufferColor.rows*occlBufferColor.cols; ++x){
			//	int status = occlBuffer[i].mat.ptr<unsigned char>()[x];
			//	cv::Vec3b color;
			//	switch (status){
			//	case OCCL_NONE:
			//		color = cv::Vec3b(50, 50, 50);
			//		break;
			//	case OCCL_OCCLUDED:
			//		color = cv::Vec3b(250, 50, 50);
			//		break;
			//	case OCCL_UNOCCLUDED:
			//		color = cv::Vec3b(59, 250, 50);
			//		break;
			//	}
			//	occlBufferColor.ptr<cv::Vec3b>()[x] = color;
			//}
			//
			//std::stringstream ss;
			//ss << "occl" << i;
			//cv::imshow(ss.str(), occlBufferColor);
		}

		//cv::waitKey();

		//cv::Mat partBufferColor(partBuffer.rows, partBuffer.cols, CV_8UC3);
		//for (int x = 0; x < partBuffer.cols*partBuffer.rows; ++x){
		//	int limb = partBuffer.ptr<unsigned char>()[x];
		//	cv::Scalar color = getLimbColor(limb, 3);
		//	partBufferColor.ptr<cv::Vec3b>()[x](0) = color(0);
		//	partBufferColor.ptr<cv::Vec3b>()[x](1) = color(1);
		//	partBufferColor.ptr<cv::Vec3b>()[x](2) = color(2);
		//}
		//
		//cv::imshow("part buffer", partBufferColor);
		//cv::imshow("color", iFullVideoFrame);
		//cv::waitKey();

		//not using part buffer any more
		//for(int x=0;x<partBuffer.cols;++x){
		//	for(int y=0;y<partBuffer.rows;++y){
		//		unsigned char i = partBuffer.at<unsigned char>(cv::Point2d(x,y));
		//		if(i<0 || i>=NUMLIMBS) continue;
		//		//cv::Point2i offset = individualLimbMat[i].offset - iVideoFrame.offset;
		//		cv::Point2i offset = individualLimbMat[i].offset;
		//		cv::Point2i pt(x,y);
		//		cv::Point2i ilmPt = pt - offset;
		//
		//
		//		if(CLAMP_SIZE(ilmPt.x, ilmPt.y, individualLimbMat[i].mat.cols, individualLimbMat[i].mat.rows))
		//			individualLimbMat[i].mat.at<cv::Vec3b>(ilmPt) = iFullVideoFrame.ptr<cv::Vec3b>(pt.y)[pt.x];
		//	}
		//}

		for(int i=0;i<NUMLIMBS;++i){

			for(int x=0;x<occlBuffer[i].mat.cols;++x){
				for(int y=0;y<occlBuffer[i].mat.rows;++y){
					unsigned char occlId = occlBuffer[i].mat.at<unsigned char>(cv::Point(x,y));
					if(occlId == OCCL_OCCLUDED){
						individualLimbMat[i].mat.at<cv::Vec3b>(cv::Point(x,y)) = cv::Vec3b(255,0,0);
					}else if(occlId == OCCL_UNOCCLUDED){
						cv::Point2i offset = individualLimbMat[i].offset;
						cv::Point2i ilmPt(x,y);
						cv::Point2i framePt = ilmPt + offset;

						if(CLAMP_SIZE(framePt.x, framePt.y, iFullVideoFrame.cols, iFullVideoFrame.rows))
							individualLimbMat[i].mat.at<cv::Vec3b>(ilmPt) = iFullVideoFrame.at<cv::Vec3b>(framePt);
					}

				}
			}

			fillHoles(individualLimbMat[i].mat.clone(), individualLimbMat[i].mat, 3);
			removeIslands(individualLimbMat[i].mat.clone(), individualLimbMat[i].mat, 3);
			framesForLimb[i].push_back(frame);
		}


		frames[frame] = (individualLimbMat);
		if(verbose) std::cout << frame << std::endl;

	}
}

void Limbrary::cluster(std::vector<SkeleVideoFrame> * vidRecord, unsigned int K, unsigned int iterations){

	if (K > vidRecord->size()) K = vidRecord->size();

	for(int limb=0;limb<NUMLIMBS;++limb)
	{
		//this is in order to make use of previously clustered data, or to skip over frames that have no image data
		bool useAllFrames = framesForLimb[limb].empty();
		int frameMax = useAllFrames?vidRecord->size():framesForLimb[limb].size();

		//KMEANS

		std::vector<cv::Mat> centers(K);
		for(int i=0;i<K;++i){

			centers[i] = jointbasedSkeletonMatrix((*vidRecord)[i%vidRecord->size()].kinectPoints2P, limb);
		}

		int iter = 0;
		while(iter < iterations){
			//assign centers to frames
		
			std::vector<cv::Mat > totals(K);
			std::vector<int> nums(K);
			for(int k=0;k<K;++k){
				totals[k] = cv::Mat(3,centers[k].cols,CV_32F,cv::Scalar(0));
				nums[k] = 0;
			}


			for(int _i=0;_i<frameMax;++_i){
				int i = useAllFrames?_i:framesForLimb[limb][_i];
				if(frames[i][limb].mat.empty()) continue;

				float dist=-1;
				int assigned;

				cv::Mat tlskel = jointbasedSkeletonMatrix((*vidRecord)[i].kinectPoints2P, limb);

				//find distance of frame from centers
				for(int j=0;j<K;++j){
					float tdist = sqrSum(tlskel-centers[j]);
					if(dist == -1 || tdist < dist){
						dist = tdist;
						assigned = j;
					}
				}

				nums[assigned]++;
				totals[assigned] += tlskel;

			}

			float delta = 0;

			//average out the totals and assign to centers
			for(int k=0;k<K;++k){
				if(nums[k] == 0) continue;

				totals[k] /= nums[k];

				//find out how much centers change
				delta += sqrSum(centers[k] - totals[k]);

				centers[k] = totals[k];
			}

			if(delta < MIN_DELTA)
				break;

			++iter;
		}

		//assign the closests to the clusters

		if(useAllFrames){
			framesForLimb[limb].clear();
			framesForLimb[limb].resize(K);
		}

		for(int k=0;k<framesForLimb.size();++k){
			float dist=-1;
			int closest;
			for(int _i=0;_i<frameMax;++_i){
				int i = useAllFrames?_i:framesForLimb[limb][_i];
	
				cv::Mat tlskel = jointbasedSkeletonMatrix((*vidRecord)[i].kinectPoints2P, limb);
				float tdist = sqrSum(tlskel-centers[k]);
				if(dist == -1 || tdist < dist){
					dist = tdist;
					closest = i;
				}
			}
			framesForLimb[limb][k] = closest;
		}

	}
}

void Limbrary::clean(){
	for(int i=0;i<NUMLIMBS;++i){
		for(int f=0;f<frames.size();++f){
			//check if framesForLimb[i] contains f
			bool contains = false;
			for(int j=0;j<framesForLimb[i].size();++j){
				if(framesForLimb[i][j] == f) contains = true;
			}
			if(!contains){
				frames[f][i].clear();
			}
		}
		for(int f=0;f<framesForLimb[i].size();++f){
			//check if frames[framesForLimb[i][f]] for i exists
			if(frames[framesForLimb[i][f]][i].mat.empty()){
				framesForLimb[i].erase(framesForLimb[i].begin()+f);
				--f;
			}
		}
	}

}

//right now, counts number of colored pixels vs number of occluded pixels; if its more than half occluded, discards that image
void Limbrary::removeBadFrames(){
	for(auto it=frames.begin(); it!=frames.end(); ++it){
		if(it->empty()) continue;
		for(int l=0;l<NUMLIMBS;++l){
			int numpix = 0, numgood = 0, numwhite = 0;
			cv::Mat m = (*it)[l].mat;
			for (int r = 0; r < m.rows; ++r){
				const cv::Vec3b * rowPtr = m.ptr<cv::Vec3b>(r);
				for (int c = 0; c < m.cols; ++c){
					const cv::Vec3b& pt = rowPtr[c];

					if (pt != cv::Vec3b(255, 255, 255)){
						++numpix;

						if (pt != cv::Vec3b(255, 0, 0)){
							++numgood;
						}
					}else{
						++numwhite;
					}

					/*
					if(m.at<cv::Vec3b>(r,c) != cv::Vec3b(255,255,255)){
					++numpix;

					if(m.at<cv::Vec3b>(r,c) != cv::Vec3b(255,0,0)){
					++numgood;
					}
					}*/
				}
			}

			if(numpix == 0 || (numgood+0.0f)/(numpix) < FL_GOOD_RATIO || (numpix+0.0f)/(numpix+numwhite) < FL_WHITE_RATIO ){
				(*it)[l] = CroppedCvMat();

			}
			else{

				float good_ratio = (numgood + 0.0f) / (numpix);
				if (good_ratio < FL_GOOD_RATIO){
					(*it)[l] = CroppedCvMat();
				}
			}
		}
	}
}

std::vector<int>& Limbrary::getAvailableFramesForLimb(int limbid)  {
	return framesForLimb[limbid];
}


void Limbrary::Save(std::string path){
	std::string filepath = path + "/limbrary.xml";
	std::string impath = "limbrary/";
	CreateDirectoryA(path.c_str(), NULL);
	CreateDirectoryA((path+impath).c_str(), NULL);

	TiXmlDocument xmlDoc;
	TiXmlDeclaration * decl = new TiXmlDeclaration("1.0", "", "");
	xmlDoc.LinkEndChild(decl);

	TiXmlElement * root = new TiXmlElement("Limbrary");
	xmlDoc.LinkEndChild(root);
	root->SetDoubleAttribute("version", 0.2);
	root->SetAttribute("numlimbs", NUMLIMBS);
	
	TiXmlElement * framesElement = new TiXmlElement("Frames");
	root->LinkEndChild(framesElement);
	framesElement->SetAttribute("numframes", frames.size());

	for(int i=0; i<frames.size(); ++i){

		if(frames[i].empty()) continue;

		TiXmlElement * frame = new TiXmlElement("Frame");
		framesElement->LinkEndChild(frame);
		frame->SetAttribute("frameid", i);
		
		for(int j=0;j<NUMLIMBS;++j){

			if(frames[i][j].mat.empty()) continue;

			TiXmlElement * limb = new TiXmlElement("Limb");
			frame->LinkEndChild(limb);
			limb->SetAttribute("limbid", j);
			limb->SetAttribute("offsetX", frames[i][j].offset.x);
			limb->SetAttribute("offsetY", frames[i][j].offset.y);
			
			limb->SetAttribute("originalWidth", frames[i][j].origWidth);
			limb->SetAttribute("originalHeight", frames[i][j].origHeight);

			std::stringstream filenameSS;
			filenameSS << impath 
				<< std::setfill('0') 
				<< "frame" << std::setw(4) << i
				<< "limb" << std::setw(2) << j
				<< ".png";

			limb->SetAttribute("filename", filenameSS.str());
			cv::imwrite(path + filenameSS.str(), frames[i][j].mat);
		}
	}

	TiXmlElement * clustersElement = new TiXmlElement("Clusters");
	root->LinkEndChild(clustersElement);
	
	for(int i=0;i<NUMLIMBS;++i){
		TiXmlElement * cluster = new TiXmlElement("Cluster");
		clustersElement->LinkEndChild(cluster);
		cluster->SetAttribute("limb", i);
		cluster->SetAttribute("numframes", framesForLimb[i].size());

		for(int j=0;j<framesForLimb[i].size();++j){
			TiXmlElement * frame = new TiXmlElement("Frame");
			cluster->LinkEndChild(frame);
			frame->SetAttribute("frameid", framesForLimb[i][j]);
		}
	}


	xmlDoc.SaveFile(filepath);
}

void Limbrary::Load(std::string path){
	std::string filepath = path + "/limbrary.xml";

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
	int numframes;
	int numlimbs;

	root->QueryDoubleAttribute("version", &version);
	root->QueryIntAttribute("numlimbs", &numlimbs);

	TiXmlHandle framesHandle(root);

	if(version == 0.1){

		root->QueryIntAttribute("numframes", &numframes);

	}else{
		
		TiXmlElement * framesElement = root->FirstChild("Frames")->ToElement();
		framesHandle = TiXmlHandle(framesElement);

		framesElement->QueryIntAttribute("numframes", &numframes);
	}

	frames.clear();
	frames.resize(numframes);

	for(int i=0;i<numframes;++i){
		frames[i].resize(numlimbs);
	}

	for(TiXmlElement * frame = framesHandle.FirstChild().ToElement();
		frame != NULL;
		frame = frame->NextSiblingElement()){
			int frameid;
			frame->QueryIntAttribute("frameid", &frameid);

			TiXmlHandle frameHandle(frame);

			for(TiXmlElement * limb = frameHandle.FirstChild().ToElement();
				limb != NULL;
				limb = limb->NextSiblingElement()){
					int limbid;
					cv::Point2i offset;

					limb->QueryIntAttribute("limbid", &limbid);
					limb->QueryIntAttribute("offsetX", &(offset.x));
					limb->QueryIntAttribute("offsetY", &(offset.y));

					frames[frameid][limbid].offset = offset;

					int ow=DEFAULT_WIDTH, oh=DEFAULT_HEIGHT;
					if(limb->Attribute("originalWidth") != NULL)  limb->QueryIntAttribute("originalWidth", &ow);
					if(limb->Attribute("originalHeight") != NULL) limb->QueryIntAttribute("originalHeight", &oh);

					frames[frameid][limbid].origWidth = ow;
					frames[frameid][limbid].origHeight = oh;

					std::string impath = limb->Attribute("filename");
					frames[frameid][limbid].mat = cv::imread(path+impath);
			}
	}

	framesForLimb.clear();
	framesForLimb.resize(NUMLIMBS);

	TiXmlNode * clustersNode = root->FirstChild("Clusters");
	if(clustersNode != NULL){

		TiXmlElement * clustersElement = clustersNode->ToElement();

		TiXmlHandle clusters(clustersElement);
		for(TiXmlElement * cluster = clusters.FirstChild().ToElement();
			cluster != NULL;
			cluster = cluster->NextSiblingElement()){
				int numframes_cl, limb;

				cluster->QueryIntAttribute("numframes", &numframes_cl);
				cluster->QueryIntAttribute("limb", &limb);

				framesForLimb[limb].clear();

				TiXmlHandle clusterHandle(cluster);
				for(TiXmlElement * clusterFrame = clusterHandle.FirstChild().ToElement();
					clusterFrame != NULL;
					clusterFrame = clusterFrame->NextSiblingElement()){
						int clusterFrameID;
						clusterFrame->QueryIntAttribute("frameid", &clusterFrameID);

						framesForLimb[limb].push_back(clusterFrameID);
				}
		}

	}
}

