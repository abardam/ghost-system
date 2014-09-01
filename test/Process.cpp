#include "ghost.h"

#define USERINPUT 0
#define VID_DIRECTORY "map000000_whitesweater_edit/video/"
#define CB_DIRECTORY "map000000-custCB/"
#define LB_LOAD_DIRECTORY "map000000_whitesweater/"
#define LB_SAVE_DIRECTORY "map000000_whitesweater_edit-custCB-clean-test/"
#define SR_DIRECTORY "map000000/estim/"
#define BUILD_CB 0
#define BUILD_LB 1
#define CLEAN_LB 1
#define CLUSTER_LB 1
#define CLUSTER_LB_N 400
#define USE_SR 0

std::vector<SkeleVideoFrame> vidRecord;
std::vector<Skeleton> wcSkeletons;
CylinderBody cylinderBody;
Limbrary limbrary;
std::vector<std::vector<char>> estimRecord;

int main(){

	std::string in;
	bool valid;

	std::string viddir, cbdir, lbloaddir, lbsavedir, srdir;
	bool buildCB, buildLB, clusterLB, cleanLB;

#if USERINPUT
	std::cout << "Video frame directory (location of SVF.xml): ";
	std::cin >> in;
	viddir = in;

	valid = false;

	while(!valid){
		std::cout << "(B)uild or (L)oad CYLINDER BODY? ";
		std::cin >> in;

		if(in == "B" || in == "b" || in == "build"){
			buildCB = true;
			valid = true;
		}
		else if(in == "L" || in == "l" || in == "load"){
			buildCB = false;
			valid = true;
		}
	}

	if(buildCB){
		std::cout << "Save ";
	}else{
		std::cout << "Load ";
	}

	std::cout << "CYLINDER BODY directory (location of cylinderbody.xml): ";
	std::cin >> in;
	cbdir = in;


	valid = false;

	while(!valid){
		std::cout << "(B)uild or (L)oad LIMBRARY? ";
		std::cin >> in;

		if(in == "B" || in == "b" || in == "build"){
			buildLB = true;
			valid = true;
		}
		else if(in == "L" || in == "l" || in == "load"){
			buildLB = false;
			valid = true;
		}
	}

	if(buildLB){
		std::cout << "Save ";
	}else{
		std::cout << "Load ";
	}

	std::cout << "LIMBRARY directory (location of limbrary.xml): ";
	std::cin >> in;
	lbdir = in;
	
	
	valid = false;

	while(!valid){
		std::cout << "Cluster LIMBRARY? ";
		std::cin >> in;

		if(in == "Y" || in == "y" || in == "yes"){
			clusterLB = true;
			valid = true;
		}
		else if(in == "N" || in == "n" || in == "no"){
			clusterLB = false;
			valid = true;
		}
	}
#else
	viddir = VID_DIRECTORY;
	cbdir = CB_DIRECTORY;
	lbloaddir = LB_LOAD_DIRECTORY;
	lbsavedir = LB_SAVE_DIRECTORY;
	srdir = SR_DIRECTORY;
	buildCB = BUILD_CB;
	buildLB = BUILD_LB;
	cleanLB = CLEAN_LB;
	clusterLB = CLUSTER_LB;
#endif
	//initAndLoad(cv::Mat::eye(4,4,CV_32F), cv::Mat::eye(4,4,CV_32F), &vidRecord, &wcSkeletons, "map000000_wall1/video/");
	//
	//buildCylinderBody(&vidRecord, &cylinderBody); //USING CUSTOM CYLINDER BODY VALUES
	////cylinderBody.Load("map000000-custCB/");
	//
	//limbrary.build(&vidRecord, &cylinderBody, true);
	////limbrary.Load("map000000/");
	//
	////cylinderBody.Save("map000000-custCB/");
	//cylinderBody.Save("map000000_wall1/");
	////limbrary.Save("map000000-custCB/");
	////limbrary.Save("map000000_wall1/");
	//
	//limbrary.cluster(&vidRecord);
	//limbrary.Save("map000000_wall1-clust/");
	////limbrary.Save("map000000-custCB-clust/");


	initAndLoad(cv::Mat::eye(4,4,CV_32F), cv::Mat::eye(4,4,CV_32F), &vidRecord, &wcSkeletons, viddir);
	
	if(buildCB){
		buildCylinderBody(&vidRecord, &cylinderBody);
		cylinderBody.Save(cbdir);
	}
	else
		cylinderBody.Load(cbdir);

	if(USE_SR){
		LoadStatusRecord(srdir, estimRecord);

		std::vector<Skeleton> interpPoints(estimRecord.size());

		for(int i=0;i<interpPoints.size();++i){
			interpPoints[i] = Skeleton(vidRecord[i].kinectPoints);
		}

		interpolate(estimRecord, interpPoints);

		for(int i=0;i<interpPoints.size();++i){
			vidRecord[i].kinectPoints = Skeleton(interpPoints[i]);
		}
	}
	
	if(buildLB)
		limbrary.build(&vidRecord, &cylinderBody, true);
	else
		limbrary.Load(lbloaddir);
	
	if(cleanLB){
		limbrary.removeBadFrames();
	}

	if(clusterLB){
		limbrary.cluster(&vidRecord, CLUSTER_LB_N);
		limbrary.clean();
	}

	limbrary.Save(lbsavedir);
	
}

