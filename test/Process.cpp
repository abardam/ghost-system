#include "ghost.h"

std::vector<SkeleVideoFrame> vidRecord;
std::vector<Skeleton> wcSkeletons;
CylinderBody cylinderBody;
Limbrary limbrary;

int main(){
	initAndLoad(cv::Mat::eye(4,4,CV_32F), cv::Mat::eye(4,4,CV_32F), &vidRecord, &wcSkeletons, "map000000/video/");

	//buildCylinderBody(&vidRecord, &cylinderBody); //USING CUSTOM CYLINDER BODY VALUES
	cylinderBody.Load("map000000-custCB/");
	
	limbrary.build(&vidRecord, &cylinderBody, true);
	//limbrary.Load("map000000/");

	//cylinderBody.Save("map000000-custCB/");
	//cylinderBody.Save("map000000/");
	limbrary.Save("map000000-custCB/");
	//limbrary.Save("map000000/");

	limbrary.cluster(&vidRecord);
	//limbrary.Save("map000000-clust/");
	limbrary.Save("map000000-custCB-clust/");

}

