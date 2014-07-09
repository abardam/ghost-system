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

#define OCCL_NONE 0
#define OCCL_UNOCCLUDED 1
#define OCCL_OCCLUDED 2

Limbrary::Limbrary():framesForLimb(NUMLIMBS){	
}

void Limbrary::build(std::vector<SkeleVideoFrame> * vidRecord, CylinderBody * cylinderBody, bool verbose){

	if(verbose) std::cout << "Building limbrary...\n";


	cv::Mat cameraMatrix = getCameraMatrix(); //for 3D pt -> 2D pt
	cv::Mat invCameraMatrix = invertCameraMatrix(cameraMatrix); //for 2D pt -> 3D ray

	
	
	for(auto it=vidRecord->begin(); it!=vidRecord->end(); ++it){
		cv::Mat draw = it->videoFrame.mat.clone();
		cv::Point2i voff = it->videoFrame.offset;

		cv::Mat zBuffer(it->videoFrame.mat.rows, it->videoFrame.mat.cols, cv::DataType<unsigned short>::type, cv::Scalar(8000));
		cv::Mat partBuffer(it->videoFrame.mat.rows, it->videoFrame.mat.cols, CV_8U, cv::Scalar(NUMLIMBS));
		CroppedCvMat occlBuffer[NUMLIMBS]; 
		FrameLimbs individualLimbMat(NUMLIMBS);


		for(int i=0;i<NUMLIMBS;++i){

			int f = getLimbmap()[i].first;
			int s = getLimbmap()[i].second;

			cv::Vec3f _a = mat_to_vec3(it->kinectPoints.points.col(f));
			cv::Vec3f _b = mat_to_vec3(it->kinectPoints.points.col(s));

			cv::Vec3f a = _b + cylinderBody->newLeftOffset_cyl[i] * (_a-_b);
			cv::Vec3f b = _a + cylinderBody->newRightOffset_cyl[i] * (_b-_a);

			
			std::vector<cv::Vec3f> fp;
			std::vector<cv::Vec3s> fpv;
			PixelPolygon p;
			cv::Mat cylPts = cylinder_to_pts(a, b, cylinderBody->newPartRadii_cyl[i], voff, &p, &fp, &fpv);
			int limbpicWidth = p.hi.size();


			cv::Point2i offset = voff + cv::Point(p.x_offset, p.lo_y);
			cv::Mat zBufferLocal = pts_to_zBuffer(cylPts, voff, offset, limbpicWidth + 1, p.hi_y - p.lo_y + 1);


			individualLimbMat[i].mat = cv::Mat(p.hi_y-p.lo_y+1,limbpicWidth+1, CV_8UC3, cv::Scalar(255,255,255));
			individualLimbMat[i].offset = voff + cv::Point2i(p.x_offset, p.lo_y);

			occlBuffer[i].mat = cv::Mat(p.hi_y-p.lo_y+1, limbpicWidth+1, CV_8U, cv::Scalar(OCCL_NONE)); //0 = no pixel, 1 = pixel unoccluded, 2 = pixel occluded
			occlBuffer[i].offset = voff + cv::Point2i(p.x_offset, p.lo_y);

			for(int x=0; x<zBufferLocal.cols; ++x){
				for(int y=0; y<zBufferLocal.rows; ++y){

					cv::Point2i zBufferLocalPixLoc(x,y);
					cv::Point2i zBufferPixLoc = zBufferLocalPixLoc - voff + offset;
					unsigned short new_depth = zBufferLocal.at<unsigned short>(zBufferLocalPixLoc); //zBufferLocal.ptr<unsigned short>(zBufferLocalPixLoc.y)[zBufferLocalPixLoc.x]; //zBufferLocal.at<unsigned short>(zBufferLocalPixLoc);

					if(new_depth == MAXDEPTH) continue;

					if(CLAMP_SIZE(zBufferPixLoc.x, zBufferPixLoc.y, zBuffer.cols, zBuffer.rows)){
						unsigned short old_depth = zBuffer.at<unsigned short>(zBufferPixLoc);
						unsigned char old_part = partBuffer.at<unsigned char>(zBufferPixLoc);
						if(old_depth > new_depth){
							zBuffer.at<unsigned short>(zBufferPixLoc) = new_depth;
							partBuffer.at<unsigned char>(zBufferPixLoc) = i;
							occlBuffer[i].mat.at<unsigned char>(zBufferLocalPixLoc) = OCCL_UNOCCLUDED;
							if(old_part != NUMLIMBS){
								cv::Point2i pixLoc_off_old = zBufferPixLoc - individualLimbMat[old_part].offset + voff;
								occlBuffer[old_part].mat.at<unsigned char>(pixLoc_off_old) = OCCL_OCCLUDED;
							}

							//color buffer assign
							//cv::circle(draw, pixLoc, 1, getLimbColor(i));
						}else{
								
							occlBuffer[i].mat.at<unsigned char>(zBufferLocalPixLoc) = OCCL_OCCLUDED;
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
		}

		for(int x=0;x<partBuffer.cols;++x){
			for(int y=0;y<partBuffer.rows;++y){
				unsigned char i = partBuffer.at<unsigned char>(cv::Point2d(x,y));
				if(i<0 || i>=NUMLIMBS) continue;
				cv::Point2i offset = -voff + individualLimbMat[i].offset;
				cv::Point2i pt(x,y);
				cv::Point2i ilmPt = pt - offset;


				if(CLAMP_SIZE(ilmPt.x, ilmPt.y, individualLimbMat[i].mat.cols, individualLimbMat[i].mat.rows))
					individualLimbMat[i].mat.at<cv::Vec3b>(ilmPt) = draw.at<cv::Vec3b>(pt);
			}
		}

		for(int i=0;i<NUMLIMBS;++i){

			for(int x=0;x<occlBuffer[i].mat.cols;++x){
				for(int y=0;y<occlBuffer[i].mat.rows;++y){
					if(occlBuffer[i].mat.at<unsigned char>(cv::Point(x,y)) == OCCL_OCCLUDED){
						individualLimbMat[i].mat.at<cv::Vec3b>(cv::Point(x,y)) = cv::Vec3b(255,0,0);
					}

				}
			}

		}

		frames.push_back(individualLimbMat);
		if(verbose) std::cout << frames.size() << std::endl;

	}
}

void Limbrary::cluster(std::vector<SkeleVideoFrame> * vidRecord, unsigned int K, unsigned int iterations){

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
		framesForLimb[limb].clear();
		framesForLimb[limb].resize(K);
		for(int k=0;k<K;++k){
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
		for(int l=0;l<NUMLIMBS;++l){
			int numpix = 0, numgood = 0;
			cv::Mat m = (*it)[l].mat;
			for(int r=0;r<m.rows;++r){
				const cv::Vec3b * rowPtr = m.ptr<cv::Vec3b>(r);
				for(int c=0;c<m.cols;++c){
					const cv::Vec3b& pt = rowPtr[c];

					if(pt != cv::Vec3b(255,255,255)){
						++numpix;

						if(pt != cv::Vec3b(255,0,0)){
							++numgood;
						}
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

			float good_ratio = (numgood+0.0f)/(numpix);
			if(good_ratio < FL_GOOD_RATIO){
				(*it)[l] = CroppedCvMat();
			}
		}
	}
}

std::vector<int> Limbrary::getAvailableFramesForLimb(int limbid) const {
	return framesForLimb[limbid];
}


FrameLimbs Limbrary::getFrameLimbs(int frame) const{
	return frames[frame];
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

