#include "ghost.h"
#include <iomanip>

std::vector<SkeleVideoFrame> vidRecord;
std::vector<Skeleton> wcSkeletons;
CylinderBody cylinderBody;
Limbrary limbrary;
int main(){
	initAndLoad(cv::Mat::eye(4,4,CV_32F),cv::Mat::eye(4,4,CV_32F),&vidRecord,&wcSkeletons,"map000000/video/",false);
	cylinderBody.Load("map000000-custCB/");
	limbrary.Load("map000000-custCB-clust/");

	cv::Vec3f translation(0,0,5);
	cv::Vec3f rotation(0,CV_PI/2,0);

	cv::Mat im(480, 640, CV_8UC3, cv::Scalar(255,255,255));
	int frame = 100;
	int r = 0;
	rotation(1) = CV_PI/2 - r * (CV_PI/16);

	
	//smoothKinectPoints(wcSkeletons, 0.5);

	while(true){
		cv::Mat draw = im.clone();
		cv::Mat transform = getTranslationMatrix(translation) * 
			mat3_to_mat4(getRotationMatrix(cv::Vec3f(1,0,0),rotation(0)) * getRotationMatrix(cv::Vec3f(0,1,0),rotation(1)) * getRotationMatrix(cv::Vec3f(0,0,1),rotation(2)));

		ghostdraw(frame, transform, vidRecord, wcSkeletons, cylinderBody, limbrary, draw);

		cv::imshow("pic", draw);
		char q = cv::waitKey(10);

		if(q=='q') return 0;
		else if(q=='w'){
			translation(2) -= 1;
		}else if(q=='s'){
			translation(2) += 1;
		}else if(q=='a'){
			rotation(1) += CV_PI/16;
		}else if(q=='d'){
			rotation(1) -= CV_PI/16;
		}

		std::string dir = "tempdir13/";
		CreateDirectoryA(dir.c_str(), NULL);

		std::stringstream ss;
		ss << dir << "frame" << std::setfill('0') << std::setw(3) << frame << "r" << std::setw(2) << r << ".png";

		cv::imwrite(ss.str(), draw);

		if(++frame >= vidRecord.size()){
			frame = 100;
			++r;
			rotation(1) = CV_PI/2 - r * (CV_PI/16);

			if(r > 16)
			{
				return 0;
			}
		}
	}
}