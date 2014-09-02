#include "ghost.h"
#include <iomanip>
#include <fstream>

#define PTAMM_STYLE 0

cv::Vec3f calcSkeleCenter(Skeleton s){

	cv::Mat sumMat = cv::Mat::ones(NUMJOINTS, 1, CV_32F);

	return mat_to_vec3( s.points * sumMat );
}

std::vector<SkeleVideoFrame> vidRecord;
std::vector<Skeleton> wcSkeletons;
std::vector<std::vector<char>> estimRecord;
CylinderBody cylinderBody;
Limbrary limbrary;
int main(){

	std::ofstream of;
	of.open("benchmark/score_out.txt");
	std::cerr.rdbuf(of.rdbuf());

	cv::Mat K2P = cv::Mat::eye(4,4,CV_32F);

#if PTAMM_STYLE == 1
	K2P = getScaleMatrix(-1,-1,1);
#endif

	initAndLoad(cv::Mat::eye(4,4,CV_32F),K2P,&vidRecord,&wcSkeletons,"map000000_whitesweater_edit/video/",false);

#if PTAMM_STYLE == 1
	LoadWorldCoordinateSkeletons(wcSkeletons, "map000000_whitesweater/GhostGame.xml");
#else
	zeroWorldCoordinateSkeletons(cv::Mat::eye(4,4,CV_32F),&vidRecord,&wcSkeletons);
#endif
	
	cv::Vec3f skeleCenter = calcSkeleCenter(wcSkeletons[0]);

	//cv::Mat shittyTransformMatrix = /*getTranslationMatrix(cv::Vec3f(0,0,1000)); */ getRotationMatrix4(cv::Vec3f(1,0,1), CV_PI);
	//cv::FileStorage fs("ptammat.yml", cv::FileStorage::READ);
	//fs["matCfW"] >> shittyTransformMatrix;
	//for(int i=0;i<wcSkeletons.size();++i){
	//	wcSkeletons[i].points = shittyTransformMatrix * wcSkeletons[i].points;
	//}

	cylinderBody.Load("map000000-custCB/");

#if PTAMM_STYLE == 1
	cylinderBody.radiusModifier = 0.658017;
#endif

	calculateSkeletonOffsetPoints(vidRecord,wcSkeletons,cylinderBody);
	//limbrary.Load("map000000_whitesweater-custCB-clust/");
	limbrary.Load("map000000_whitesweater_edit-custCB-clean-test/");

	//LoadStatusRecord("map000000_whitesweater/estim/", estimRecord);
	//std::vector<Skeleton> wcSkeletonsEstim = wcSkeletons;
	//interpolate(estimRecord, wcSkeletonsEstim);

	cv::Vec3f translation(0,0,0);
	cv::Vec3f rotation(0,CV_PI/2,0);


	cv::Mat im(480, 640, CV_8UC4, cv::Scalar(255,255,255,0));
	int frame = 100;
	int r = 8;
	int maxr = 16-4;
	rotation(1) = CV_PI/2 - r * (CV_PI/16);

	int p = 8;
	rotation(0) = CV_PI/2 - (CV_PI/16) * p;

	rotation(2) = 0;

	//smoothKinectPoints(wcSkeletons, 0.5);

	bool play = true;
	while(true){

		{

			unsigned char options = GD_DRAW ;// | GD_NOWEIGHT | GD_NOLIMBRARY;
			cv::Mat draw = im.clone();
			cv::Mat zBuf;

			cv::Mat transform = getTranslationMatrix(translation) * getTranslationMatrix(skeleCenter) *
				mat3_to_mat4(getRotationMatrix(cv::Vec3f(1,0,0),rotation(0)) * getRotationMatrix(cv::Vec3f(0,1,0),rotation(1)) * getRotationMatrix(cv::Vec3f(0,0,1),rotation(2)))
				* getTranslationMatrix(-skeleCenter);

#if PTAMM_STYLE == 1
			cv::FileStorage fs("ptammat.yml", cv::FileStorage::READ);
			fs["matCfW"] >> transform;
		
			transform = getTranslationMatrix(translation) * getTranslationMatrix(skeleCenter) *
				mat3_to_mat4(getRotationMatrix(cv::Vec3f(1,0,0),rotation(0)) * getRotationMatrix(cv::Vec3f(0,1,0),rotation(1)) * getRotationMatrix(cv::Vec3f(0,0,1),rotation(2)))
				* getTranslationMatrix(-skeleCenter) * transform;
#endif

			ScoreList scoreList[NUMLIMBS];


			ghostdraw_parallel(frame, K2P.inv() * transform /* shittyTransformMatrix.inv()*/, vidRecord, wcSkeletons, cylinderBody, limbrary, draw, zBuf, options, scoreList);

			/*
			//hybrid
			cv::Mat draw2 = im.clone();
			options = GD_CYL ;
		
			ghostdraw_parallel(frame, transform, vidRecord, wcSkeletons, cylinderBody, limbrary, draw2, options);

			cv::Mat candidate = vidRecord[scoreList[0].front().first].videoFrame.mat;

			cv::Mat cand_resize(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(200,200,200));
			int gap_y = (HEIGHT-candidate.rows)/2;
			int gap_x = (HEIGHT-candidate.cols)/2;
			//candidate.copyTo(cand_resize(cv::Range(gap_y,gap_y + candidate.rows), cv::Range(gap_x,gap_x+candidate.cols)));
			candidate.copyTo(draw(cv::Range(0,candidate.rows), cv::Range(0,candidate.cols)));
			cv::rectangle(draw,cv::Rect(0,0,candidate.cols,candidate.rows),cv::Scalar(200,150,100));

			cv::Mat combine(HEIGHT*2,WIDTH,CV_8UC3);

			draw2.copyTo(combine(cv::Range(0,draw2.rows),cv::Range(0,draw2.cols)));
			draw.copyTo(combine(cv::Range(draw2.rows,draw.rows+draw2.rows),cv::Range(0,draw.cols)));
			//cand_resize.copyTo(combine(cv::Range(draw.rows+draw2.rows,draw.rows+draw2.rows+cand_resize.rows),cv::Range(0,cand_resize.cols)));*/

		
			//hybrid
			//cv::Mat draw2 = im.clone();
			//
			//ghostdraw_parallel(frame, transform, vidRecord, wcSkeletonsEstim, cylinderBody, limbrary, draw2, zBuf, options);
			//
			//cv::Mat combine(HEIGHT*2,WIDTH,CV_8UC4);
			//
			//draw2.copyTo(combine(cv::Range(0,draw2.rows),cv::Range(0,draw2.cols)));
			//draw.copyTo(combine(cv::Range(draw2.rows,draw.rows+draw2.rows),cv::Range(0,draw.cols)));


			cv::imshow("pic", draw);

			cv::Mat zBufNorm;
			cv::normalize(zBuf, zBufNorm, 0, 255, cv::NORM_MINMAX);

			cv::Mat zBuf8(zBuf.rows, zBuf.cols, CV_8U);
			for(int i=0;i<zBuf.rows*zBuf.cols;++i){
				zBuf8.ptr<unsigned char>()[i] = 255-zBufNorm.ptr<unsigned short>()[i];
			}

			cv::imshow("depth", zBuf8);

			std::string dir = "out_ws_interpcmp/";
			CreateDirectoryA(dir.c_str(), NULL);

			std::stringstream ss;
			ss << dir << 
				"frame" << std::setfill('0') << std::setw(3) << frame << 
				"r" << std::setw(2) << r <<
				"p" << p << ".png";

			//cv::imwrite(ss.str(), draw);

			if(play)
			{
				if(++frame >= vidRecord.size()){
					frame = 100;
					++r;
					rotation(1) = CV_PI/2 - r * (CV_PI/16);

					if(r > maxr)
					{
						return 0;
					}
				}
			}
		}

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
		}else if(q=='z'){
			rotation (2) += CV_PI/64;
		}else if(q=='x'){
			rotation (2) -= CV_PI/64;
		}else if(q=='p'){
			play = !play;
		}

		
	}
}