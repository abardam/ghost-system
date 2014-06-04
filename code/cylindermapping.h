#pragma once

#include "definitions.h"

#ifdef GH_CMAPPING

#include <opencv2\opencv.hpp>
#include "CylinderBody.h"


struct DrawOptions{
	bool drawCylinders;
	bool colorPixels;
	bool drawOnPic;
	bool noRot;
	bool writecmp;
	bool usePose;
	bool indivLimbs;
	bool shittyOption;
	bool grid;
	bool vidOut;
	bool forceBest;
	bool individualRender;
	DrawOptions(){
		vidOut=false;
		forceBest=false;
	};
};

void drawSkeletonCylinder(cv::Mat rotmat, int frame, std::vector<SkeleVideoFrame> * vidRecord, std::vector<Skeleton> * wcSkeletons, CylinderBody * cb, int * chosenBest, std::string * outstring);
void calcInvProjMat(int w, int h);
void glInit();
void glDestroy();
void glTexMat(cv::Mat texmat, bool fullscreen);

void depth2points(const cv::Mat &d, cv::Mat *mP);
std::vector<VecPart> depth2points_colorbased(const cv::Mat &d, const cv::Mat &r, std::vector<cv::Vec2f> &pts);
void depth2points_matarr(const cv::Mat &d, const cv::Mat &r, cv::Mat * mP);

//takes in a 4xN mat (cols are 4D pts) e.g. from depth2pts
//returns a 2xN mat, cols are 2D pixel locations of the 4D points
//requires an inverse projection matrix (of the original points) and a forward projection matrix (onto the image)
//inverse projection matrix can be acquired by the glGetFloatv and is 4x4
//forward projection matrix is 2x3 [fx 0 cx; 0 fy cy]
cv::Mat projectOnImage(cv::Mat _4D, cv::Mat invProj, cv::Mat forProj);

#endif