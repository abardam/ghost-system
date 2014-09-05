#include <Windows.h>

#include "definitions.h"

#ifdef GH_CMAPPING

#include "cylindermapping.h"
#include "cvutil.h"
#include "ghostutil.h"
#include "loader.h"
#include "cylinder.h"
#include "CylinderBody.h"
#include "KinectManager.h"
#include "ghostcam.h"

#include "opencv2\opencv.hpp"
#include <TooN\TooN.h>
#include <gvars3\instances.h>

#include <time.h>
#include <sstream>

#define CYL_DOT 2
#define COLORMAPPART (1./NUMLIMBS)

using GVars3::gvar3;
using GVars3::HIDDEN;
using GVars3::SILENT;

cv::Mat projMat;
cv::Mat invProjMat;

int pixsize = 1;
int npixsize = 1;

GLUquadricObj *quadric;

int window_width, window_height;


void glInit(){
	
	
	quadric=gluNewQuadric();
	gluQuadricNormals(quadric, GLU_SMOOTH);
}

void glDestroy(){
	
	gluDeleteQuadric(quadric);
}


//uno's

void depth2points(const cv::Mat &d, cv::Mat *mP){
	const float *dp = d.ptr<float>();
	int w = d.size().width;
	int h = d.size().height;

	int cnt=0;
	for(int i=0; i<w*h; i += npixsize) if(dp[i] < 1.0) ++cnt;

	std::vector<cv::Vec4f> P(cnt);

	const float cx = 2./w;
	const float cy = 2./h;

	int c = 0;
	for(int y = 0; y < h; y+=npixsize){
		for(int x = 0; x < w; x+=npixsize){
			float z = dp[x+y*w];
			if(z <1.){
				P[c] = (cv::Vec4f(
					cx*x-1,
					cy*y-1,
					2*z-1,
					1));

				++c;
			}

			if(z < 0){
				std::cout << z << std::endl;
			}
		}
	}
	mP->create(4,P.size(),cv::DataType<float>::type);

	for(int j=0;j<4;++j){
		for(int k=0;k<P.size();++k){
			mP->at<float>(j,k) = P[k][j];
		}
	}
};

std::vector<VecPart> depth2points_colorbased(const cv::Mat &d, const cv::Mat &r, std::vector<cv::Vec2f> &pts){
	const float *dp = d.ptr<float>();
	const float *rp = r.ptr<float>();
	int w = d.size().width;
	int h = d.size().height;

	int cnt=0;
	for(int i=0; i<w*h; i += npixsize) if(dp[i] < 1.0) ++cnt;

	std::vector<VecPart> P(cnt);
	pts.resize(cnt);

	const float cx = 2./w;
	const float cy = 2./h;

	int c = 0;
	for(int y = 0; y < h; y+=npixsize){
		for(int x = 0; x < w; x+=npixsize){
			float z = dp[x+y*w];
			if(z <1.){
				pts[c][0] = x;
				pts[c][1] = y;
				P[c].vec = (cv::Vec4f(
					cx*x-1,
					cy*y-1,
					2*z-1,
					1));
				P[c].part = rp[x+y*w] / COLORMAPPART;
				if(P[c].part >= NUMLIMBS) P[c].part = 0;

				++c;
			}

			if(z < 0){
				std::cout << z << std::endl;
			}
		}
	}

	return P;
};

void depth2points_matarr(const cv::Mat &d, const cv::Mat &r, cv::Mat * mP){
	const float *dp = d.ptr<float>();
	const float *rp = r.ptr<float>();
	int w = d.size().width;
	int h = d.size().height;

	std::vector<cv::Vec4f> P [NUMLIMBS];

	const float cx = 2./w;
	const float cy = 2./h;

	int c = 0;
	int pt;
	for(int y = 0; y < h; y+=npixsize){
		for(int x = 0; x < w; x+=npixsize){
			float z = dp[x+y*w];
			if(z <1.){
				pt = rp[x+y*w] / COLORMAPPART;

				if(pt >= NUMLIMBS) pt = 0;
				P[pt].push_back(cv::Vec4f(
					cx*x-1,
					cy*y-1,
					2*z-1,
					1));
				++c;
			}

			if(z < 0){
				std::cout << z << std::endl;
			}
		}
	}
	for(int i=0;i<NUMLIMBS;++i){
		mP[i].create(4,P[i].size(),cv::DataType<float>::type);

		for(int j=0;j<4;++j){
			for(int k=0;k<P[i].size();++k){
				mP[i].at<float>(j,k) = P[i][k][j];
			}
		}
	}
}

//takes in a 4xN mat (cols are 4D pts) e.g. from depth2pts
//returns a 2xN mat, cols are 2D pixel locations of the 4D points
//requires an inverse projection matrix (of the original points) and a forward projection matrix (onto the image)
//inverse projection matrix can be acquired by the glGetFloatv
cv::Mat projectOnImage(cv::Mat _4D, cv::Mat invProj, cv::Mat forProj){
	if(_4D.cols == 0) return cv::Mat();
	cv::Mat unproj_4D = invProj * _4D;
	for(int c=0;c<unproj_4D.cols;++c){
		unproj_4D.at<float>(0,c) /= unproj_4D.at<float>(2,c);
		unproj_4D.at<float>(1,c) /= unproj_4D.at<float>(2,c);
		unproj_4D.at<float>(2,c) /= unproj_4D.at<float>(2,c);
	}
	cv::Mat _2D = forProj * unproj_4D.rowRange(0,3);
	return _2D;
}


void calcInvProjMat(int w, int h){
	projMat = cv::Mat(4, 4, CV_32FC1);
	glGetFloatv(GL_PROJECTION_MATRIX, projMat.ptr<float>());
	projMat = projMat.t();

	//std::cout << "proj mat: \n";
	//std::cout << projMat << std::endl;
	
	cv::invert(projMat, invProjMat);

	window_width = w;
	window_height = h;
}

IMGPIXEL getColorFromPixel(int frame, cv::Vec3f pt, int part, cv::Vec2f * pixelLoc, CylinderBody * cb ){
	//glColor3f(1,1,pt[2]/2.f);

	return cb->getColorAtPartAndPixel(frame, part, cv::Vec4f(pt[0], pt[1], pt[2], 1), pixelLoc);

}

int partSwap(int i){
	if(i == UPPERARM_LEFT)	return UPPERARM_RIGHT;
	if(i == UPPERARM_RIGHT) return UPPERARM_LEFT;
	if(i == UPPERLEG_LEFT)	return UPPERLEG_RIGHT;
	if(i == UPPERLEG_RIGHT) return UPPERLEG_LEFT;
	if(i == LOWERARM_LEFT)	return LOWERARM_RIGHT;
	if(i == LOWERARM_RIGHT) return LOWERARM_LEFT;
	if(i == LOWERLEG_LEFT)	return LOWERLEG_RIGHT;
	if(i == LOWERLEG_RIGHT) return LOWERLEG_LEFT;
	return i;
}


void drawSkeletonOn(cv::Vec3f * pts, cv::Mat img, cv::Point2d offset){
	cv::Vec2f offsetV(offset.x, offset.y);
	for(int i=0;i<NUMLIMBS;++i){
		lineAt(img, toScreen(pts[getLimbmap()[i].first])-offsetV, toScreen(pts[getLimbmap()[i].second])-offsetV, IMGPIXEL(0,255,0,255));
	}
}

SYSTEMTIME lastTime;
int numFPS = 0;
float currAveFPS = 0;

void drawSkeletonCylinder(cv::Mat rotmat, int frame, std::vector<SkeleVideoFrame> * vidRecord, std::vector<Skeleton> * wcSkeletons, CylinderBody * cb, int * chosenBest, std::string * outstring){
	
	if(vidRecord->size() < 1) return;
	if(window_height == 0 || window_width == 0){
		std::cerr << "error: run calcInvProjMat() first!\n";
	}

	//npixsize = pixsize;
	npixsize = 1;
	
	std::stringstream ss;
	std::stringstream FPSss;

	//check FPS

	SYSTEMTIME thisTime;
	GetSystemTime(&thisTime);
	double seconds = thisTime.wSecond - lastTime.wSecond + (thisTime.wMilliseconds - lastTime.wMilliseconds)/1000.;
	lastTime = thisTime;
	double FPSnow = 1./seconds;

	FPSss << "FPS: " << FPSnow;
	
	double sumFPS = 0;
	sumFPS = currAveFPS*numFPS;

	sumFPS += FPSnow;
	++numFPS;

	currAveFPS = sumFPS/numFPS;
	
	FPSss << " average: " << currAveFPS;
	
    static gvar3<int> gvDrawLines("GH_DrawCylinders", 0, HIDDEN|SILENT);
    static gvar3<int> gvDrawCylinders("GH_DrawCylinders", 0, HIDDEN|SILENT);
    static gvar3<int> gvColorPixels("GH_ColorPixels", 0, HIDDEN|SILENT);
    static gvar3<int> gvDrawOnPic("GH_DrawOnPic", 0, HIDDEN|SILENT);
    static gvar3<int> gvNoRot("GH_NoRotation", 0, HIDDEN|SILENT);
    static gvar3<int> gvWriteCmp("GH_WriteCmp", 0, HIDDEN|SILENT);
    static gvar3<int> gvUsePose("GH_UsePose", 0, HIDDEN|SILENT);
    static gvar3<int> gvIndivLimbs("GH_IndividualLimbs", 0, HIDDEN|SILENT);
    static gvar3<int> gvIndividualRender("GH_IndividualRender", 0, HIDDEN|SILENT);
    static gvar3<int> gvShittyOption("GH_ShittyOption", 0, HIDDEN|SILENT);


	//bool normcyl = true;
	bool whitePixels = false;
	bool pout = false;
	//bool colorPixels = false; //make this true
	
	bool renderCylinders = true;
	//bool drawOnPic = false;
	bool frame0 = false;
	//bool drawCylinder = true; //make this false


	unsigned char * cp;
	cp = (unsigned char*) malloc(window_width*window_height*sizeof(unsigned char)*3);
	glReadPixels(0, 0, window_width, window_height, GL_BGR, GL_UNSIGNED_BYTE, cp);

	cv::Mat origTexture_ = cv::Mat(window_height, window_width, cv::DataType<IMGPIXEL>::type, cp).clone();
	cv::Mat origTexture;

	cv::resize(origTexture_, origTexture, cv::Size(640, 480)); //idk. probably should fix this

	free(cp);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_ALPHA_TEST);
	glDisable(GL_POINT_SMOOTH);
	glEnable (GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glClearColor (0.5,0.0,0.0,1.0);
	glClear (GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glPushMatrix();

		if(frame0) frame = 0;

	
		cv::Mat rotmat_t = rotmat.t();

		glMultMatrixf(rotmat_t.ptr<float>());



		cv::Mat rotmat_r = rotmat.inv();
		cv::Mat unrotmat;

		unrotmat = rotmat_r * invProjMat;

		glColor3f (1,1,1);
		glPointSize(19);

		//SkeleVideoFrame currentSVF = (*vidRecord)[(frame)%vidRecord->size()]; 
		Skeleton currentSkeleton = (*wcSkeletons)[(frame)%vidRecord->size()]; 

		//Vector4 * skele = currentSVF.kinectPoints;
	
		cv::Mat skeletonPositions4 = currentSkeleton.points.clone();
		
		cv::Mat transformedSkeletonPositions = rotmat * skeletonPositions4;

		if(*gvDrawLines)
		{

			glLineWidth(5);
			glBegin(GL_LINES);

			glColor3b(-127,-127,127);
			for(int i=0;i<NUMLIMBS;++i){
				int f = getLimbmap()[i].first;
				glVertex3f(skeletonPositions4.at<float>(0,f),skeletonPositions4.at<float>(1,f),skeletonPositions4.at<float>(2,f));
				int s = getLimbmap()[i].second;
				glVertex3f(skeletonPositions4.at<float>(0,s),skeletonPositions4.at<float>(1,s),skeletonPositions4.at<float>(2,s));
			}
			/*
			int cj = KINECT::getCenterJoint();

			glColor3b(127,127,127);
			glVertex3f(skeletonPositions4.at<float>(0,cj), 
				skeletonPositions4.at<float>(1,cj), 
				skeletonPositions4.at<float>(2,cj));
			
			cv::Vec3f newf = vec4_to_vec3( skeletonPositions4.col(cj) ) + currentSVF.facing;
			//cv::Vec3f newf2 = vec4_to_vec3( skeletonPositions4[NUI_SKELETON_POSITION_SHOULDER_CENTER] ) - currentSVF.facing;

			glVertex3f(newf[0], newf[1], newf[2]);
			//glVertex3f(newf2[0], newf2[1], newf2[2]);
			*/

			glEnd();

		}
		
		clock_t clk1;
		clock_t clk2;

		clk1 = clock();

		int bestFrame;
		int bestFrames[NUMLIMBS];



		if(!*gvIndivLimbs)
		{
			if(/*!d.forceBest <-- not too sure what this did*/ true)
			{
				//if(!*gvUsePose) //not sure i want to use this
				if(false)
				{
			
					//calculate the facing vector that the viewer sees
					cv::Vec3f viewfacing_ = mat_to_vec3( rotmat * vec3_to_mat4(vec4_to_vec3 (skeletonPositions4.col(KINECT::getCenterJoint())) + KINECT::calculateFacing(&currentSkeleton)));
					cv::Vec3f viewcshoulder_ = mat_to_vec3(rotmat * cv::Mat(skeletonPositions4.col(KINECT::getCenterJoint())));
					cv::Vec3f viewfacing = viewfacing_ - viewcshoulder_;

					bestFrame = cb->getBestFrame(viewfacing, frame, *gvWriteCmp);
				}
				else
				{
					bestFrame = cb->getBestFrame(transformedSkeletonPositions, (bool)(*gvWriteCmp));
				}

				*chosenBest = bestFrame;
			}else{
				

				bestFrame = *chosenBest;
				std::cout << "score: " << calculateScore((*vidRecord)[bestFrame].kinectPoints2P, normalizeSkeleton(transformedSkeletonPositions)) << " ";
			}
		}else{
			cb->getBestFrames(transformedSkeletonPositions, bestFrames);
		}

		clk2 = clock();

		float cmp = clk2-clk1;
		ss << " calculate best frame: " << cmp;

		cv::Mat toColorPic;
		
		if(!*gvIndivLimbs)
			toColorPic = (*vidRecord)[bestFrame].videoFrame.mat.clone();
		//cv::Point2d toColorOffset = (*vidRecord)[bestFrame].videoFrame.offset;
		//bestFrame = 4;
		//std::vector<std::pair<IMGPIXEL, cv::Vec3f>> colorPointVector;

		

		//calculate drawing order
		int order[NUMLIMBS];
		cv::Vec3f centers[NUMLIMBS];

		for(int i=0;i<NUMLIMBS;++i){
			int f = getLimbmap()[i].first;
			int s = getLimbmap()[i].second;
			cv::Vec3f in = (mat_to_vec3(transformedSkeletonPositions.col(f))+mat_to_vec3(transformedSkeletonPositions.col(s)))/2;

			int j=0;
			while(j<i){

				if(centers[j](2) < in(2)){
					break;
				}

				++j;
			}
			
			//insert into order at index j
			for(int k=NUMLIMBS-1;k>j;--k){
				order[k] = order[k-1];
				centers[k] = centers[k-1];
				
			}
			order[j] = i;
			centers[j] = in;
		}

		clk1 = clock();

		cv::Mat unrotmat2[NUMLIMBS];
		cv::Mat pt3d_mat[NUMLIMBS];

#if 0
		for (int j=0;j<getCombineParts().size();++j){
			int i = getCombineParts()[j][0];
#else
		for(int j=0;j<NUMLIMBS;++j){
			int i=order[j];
#endif

			int f = getLimbmap()[i].first;
			int s = getLimbmap()[i].second;
			
			cv::Vec4f first_ = skeletonPositions4.col(f);
			cv::Vec4f second_ = skeletonPositions4.col(s);

			cv::Vec4f first = first_; //+ (first_ - second_) * -getLeftOffset()[j];
			cv::Vec4f second = second_; //+ (second_ - first_) * getRightOffset()[j];

			//renderCylinder_convenient(skele[f].x,skele[f].y,skele[f].z,skele[s].x,skele[s].y,skele[s].z, partRadii[i], 15);

			glPushMatrix();
#if GH_MODELFITTING == GH_MF_OLD
				//cv::Mat transformMat_t = (  calcTransform_r( vec4_to_vec3(first), vec4_to_vec3(second)) * getScaleMatrix(getPartRadii()[i], getPartRadii()[i], 1)).t();
				cv::Mat transformMat_t = (  getCylinderTransform( vec4_to_vec3(first), vec4_to_vec3(second), cb->partRadii[i], cb->leftOffset[i], cb->rightOffset[i])).t();
#elif GH_MODELFITTING == GH_MF_CYLPROJ
				cv::Mat transformMat_t = (  getCylinderTransform( vec4_to_vec3(first), vec4_to_vec3(second), cb->newPartRadii_cyl[i], cb->newLeftOffset_cyl[i], cb->newRightOffset_cyl[i])).t();
#endif
				//cv::Mat unrotmat2 = getScaleMatrix(1/getPartRadii()[i], 1/getPartRadii()[i], 1) * calcTransform( vec4_to_vec3(first), vec4_to_vec3(second)) *  unrotmat;
		
				cv::Mat _1sttrans = transformMat_t.t().inv();
				//find facing vector 

				//cv::Vec3f facing = mat2Vec3(  _1sttrans * vec2HomoMat( getBestFacingVector( vec4_to_vec3(first), vec4_to_vec3(second))+ vec4_to_vec3(first)) );

				cv::Vec3f realVectorA = mat_to_vec3( /*projMat */ rotmat_t.t() *  cv::Mat(first));
				cv::Vec3f realVectorB = mat_to_vec3(/*projMat */rotmat_t.t() * cv::Mat(second));

				cv::Vec3f bestFacingVector = getBestFacingVector( realVectorA, realVectorB)+ realVectorB;

				cv::Vec3f facing = mat_to_vec3(   _1sttrans*rotmat_r /* invProjMat*/ *  vec3_to_mat4( bestFacingVector   ) );
				facing(2) = 0;

				//find rotation to face front

				cv::Mat rot2Front = mat3_to_mat4( getRotationMatrix(cv::Vec3f(0, -1, 0), facing ) );
				
				unrotmat2[i] = _1sttrans * unrotmat;

				if(!*gvNoRot) 
					unrotmat2[i] = rot2Front * unrotmat2[i];

				glMultMatrixf(transformMat_t.ptr<float>());

				float color_part = COLORMAPPART*i;

				if( *gvDrawCylinders){
					glColor4f(color_part,color_part,color_part,1);
				}else{
					glColor4f(color_part,color_part,color_part,0);
				}

				if(renderCylinders && *gvDrawCylinders)
				{
					renderCylinder(0,0,0,0,0,1, 1,1, 15,quadric);
				}

				// unify the depth buffer reading; colors correspond to parts
				//NOTE: if we want to calculate the cylinders for each part individually (i.e. to preserve transparency) this next part needs to be IN THE LOOP
				//otherwise, it needs to be OUT OF IT (up until the glClear)
				//lets make a DrawOption with that

				if(*gvIndividualRender)
				{

					float * dp;
					dp = (float*) malloc(window_width*window_height*sizeof(float));
					glReadPixels(0, 0, window_width, window_height, GL_DEPTH_COMPONENT, GL_FLOAT, dp);


					depth2points(cv::Mat(window_height, window_width, cv::DataType<float>::type, dp).clone(), &pt3d_mat[i]);

					free(dp);

					/*if(!drawCylinder)*/ glClear( GL_DEPTH_BUFFER_BIT);
				}
			glPopMatrix();

		}

		clk2 = clock();
		cmp = clk2 - clk1;

		ss << " calc transform (all parts): " << cmp;

		clk1 = clock();

		cv::Mat texmat = origTexture;

		if(!*gvIndividualRender){
			float * dp;
			dp = (float*) malloc(window_width*window_height*sizeof(float));
			glReadPixels(0, 0, window_width, window_height, GL_DEPTH_COMPONENT, GL_FLOAT, dp);
			
			float * rp;
			rp = (float*) malloc(window_width*window_height*sizeof(float));
			glReadPixels(0, 0, window_width, window_height, GL_RED, GL_FLOAT, rp);


			depth2points_matarr(cv::Mat(window_height, window_width, cv::DataType<float>::type, dp).clone(),
								cv::Mat(window_height, window_width, cv::DataType<float>::type, rp).clone(),
								pt3d_mat);

			free(dp);
			free(rp);
		}


		
		if(!*gvDrawCylinders) glClear( GL_DEPTH_BUFFER_BIT );
		if(*gvColorPixels){
			if(!(bool)(*gvIndivLimbs))
				cb->colorsAtPixels(pt3d_mat, unrotmat2, bestFrame, &texmat, order, *gvDrawOnPic || *gvWriteCmp ? &toColorPic : NULL, whitePixels, *gvShittyOption);
			else
				cb->colorsAtPixelsMultiFrame(pt3d_mat, unrotmat2, bestFrames, &texmat);
		}

		clk2 = clock();
		cmp = clk2 - clk1;

		ss << " colorsAtPixels: " << cmp;

		
		if(*gvDrawOnPic || *gvWriteCmp){

			CreateDirectoryA("shit2", NULL);

			//drawSkeletonOn(currentSVF.kinectPoints, toColorPic, toColorOffset);
			std::stringstream ss;
			ss << "shit2/" << bestFrame << ".png";

			cv::imwrite(ss.str(), toColorPic);
			*gvDrawOnPic = false;
		}

		
		clk1 = clock();

		if(*gvColorPixels)
		{
			glTexMat(texmat, true);

		}

		
		clk2 = clock();
		cmp = clk2 - clk1;

		ss << " texturing: " << cmp;

	glPopMatrix();
	//glFlush();
	//glutSwapBuffers();
	
	//*outstring = ss.str();
	*outstring = FPSss.str();

	//record video
	if(/*d.vidOut <-- defaulted to false*/ false)
	{

		static int vidID = 0;
	
		cp = (unsigned char*) malloc(window_width*window_height*sizeof(unsigned char)*3);
		glReadPixels(0, 0, window_width, window_height, GL_BGR, GL_UNSIGNED_BYTE, cp);

		cv::Mat glTex = cv::Mat(window_height, window_width, cv::DataType<IMGPIXEL>::type, cp).clone();

		free(cp);

		std::stringstream ss;
		ss << "out/vid" << vidID << ".png";

		cv::imwrite(ss.str(), glTex);

		if (*gvDrawCylinders){
			ss.clear();
			ss << "out/best" << vidID << ".png";
			cv::imwrite(ss.str(), (*vidRecord)[*chosenBest].videoFrame.mat);
		}

		//cv::imwrite("out/cyl"+innum+".png", cylImg);

		++vidID;
	}
}

void glTexMat(cv::Mat texmat, bool fullscreen){
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

		glLoadIdentity();

		/*
		if(normcyl) {
			glTranslatef(0,0,-5);
			//glRotatef(90, 1,0,0);
		}

		glPointSize(npixsize);
		glBegin(GL_POINTS);

		for(auto it = colorPointVector.begin(); it != colorPointVector.end(); ++it){
			glColor3ub(it->first[2], it->first[1],it->first[0]);
			glVertex3f(it->second[0], it->second[1], it->second[2]);
		}

		glEnd();*/


		GLuint texture[1];

		//load in cv mat as texture
		glEnable( GL_TEXTURE_2D );
	
		if(texmat.data == NULL){
			std::cout << "failed to load texture...\n";
		}else{
			glGenTextures(1, texture);


			glBindTexture(GL_TEXTURE_2D, texture[0]);

				
			// select modulate to mix texture with color for shading
			glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );

			// when texture area is small, bilinear filter the closest mipmap
			glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,GL_LINEAR); 

			// if wrap is true, the texture wraps over at the edges (repeat)
			//       ... false, the texture ends at the edges (clamp)
			glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S , GL_REPEAT );
			glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );

			glTexImage2D(GL_TEXTURE_2D, 0, 4, texmat.cols, texmat.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, texmat.data);

			glColor3f(1,1,1);


			if(fullscreen){
				glBegin( GL_QUADS );
				glTexCoord2d(0.0,0.0); glVertex2d(-1.0,-1.0);
				glTexCoord2d(1.0,0.0); glVertex2d(1.0,-1.0);
				glTexCoord2d(1.0,1.0); glVertex2d(1.0,1.0);
				glTexCoord2d(0.0,1.0); glVertex2d(-1.0,1.0);
				glEnd();
			}
			else{

			}

			glDeleteTextures( 1, texture );

		}



	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}

#endif