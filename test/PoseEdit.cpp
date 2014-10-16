#include <opencv2\opencv.hpp>
#include <ghost.h>
#include <camlerp.h>

std::vector<SkeleVideoFrame> vidRecord;
std::vector<Skeleton> wcSkeletons;
CylinderBody cylinderBody;
cv::Rect boundingBoxLerp;
LerpCorners lc;

const int HEIGHT = 480;
const int WIDTH = 640;

int frame;
float angle_y;
float angle_x;
cv::Mat trans;

bool alljoints = false;;

void onMouse(int e, int x, int y, int, void *);

cv::Vec3f calcSkeleCenter(Skeleton s){

	cv::Mat sumMat = cv::Mat::ones(NUMJOINTS, 1, CV_32F);

	return mat_to_vec3( s.points * sumMat );
}

int main(){

	initAndLoad(cv::Mat::eye(4,4,CV_32F), cv::Mat::eye(4,4,CV_32F), &vidRecord, &wcSkeletons, "map000000_aoto2/video/", true);
	zeroWorldCoordinateSkeletons(cv::Mat::eye(4,4,CV_32F), &vidRecord, &wcSkeletons);
	setCameraMatrixTexture(KINECT::loadCameraParameters());
	setCameraMatrixScene(KINECT::loadCameraParameters());
	cylinderBody.Load("map000000-custCB/");

	cv::namedWindow("rgb");
	cv::namedWindow("3d", CV_GUI_NORMAL);
	cv::setMouseCallback("3d", onMouse);

	frame=0;
	angle_y = 0;
	angle_x = 0;
	trans = cv::Mat::eye(4,4,CV_32F);

	calculateSkeletonOffsetPoints(vidRecord, wcSkeletons, cylinderBody);
	boundingBoxLerp = cv::Rect (0,0,WIDTH,HEIGHT);
	lc = generateLerpCorners(boundingBoxLerp);

	while(true){
		cv::Mat tex_ = uncrop(vidRecord[frame].videoFrame);
		cv::Mat tex(tex_.size(), CV_8UC4, cv::Scalar(255,255,255,255));
		int from_to[] = { 0,0, 1,1, 2,2};
		cv::mixChannels(&tex_,1,&tex,1,from_to,3);

		cv::Mat _3d(HEIGHT, WIDTH, CV_8UC4, cv::Scalar(0,0,0,255));

		cv::Vec3f sCenter = calcSkeleCenter(vidRecord[frame].kinectPoints);
		trans = getTranslationMatrix(sCenter) * mat3_to_mat4(getRotationMatrix(cv::Vec3f(0,1,0), angle_y)) * mat3_to_mat4(getRotationMatrix(cv::Vec3f(1,0,0), angle_x)) * getTranslationMatrix(-sCenter);

		ghostdraw_parallel(frame, cv::Mat::eye(4,4,CV_32F), vidRecord, wcSkeletons, cylinderBody, Limbrary(), tex, cv::Mat(), GD_CYL);
		ghostdraw_parallel(frame, trans, vidRecord, wcSkeletons, cylinderBody, Limbrary(), _3d, cv::Mat(), GD_CYL);

		for(int joint=0;joint<NUMJOINTS;++joint)
		{
			{
				cv::Vec3f jv = mat_to_vec3(wcSkeletons[frame].points.col(joint));
				cv::Vec2f jv2 = mat4_to_vec2(getCameraMatrixScene() * vec3_to_mat4(jv));
				cv::Point pj(jv2(0), jv2(1));

				cv::circle(tex,pj,6,cv::Scalar(50,200,250),-1);
			}

			{
				cv::Vec3f jv = mat_to_vec3(trans * wcSkeletons[frame].points.col(joint));
				cv::Vec2f jv2 = mat4_to_vec2(getCameraMatrixScene() * vec3_to_mat4(jv));
				cv::Point pj(jv2(0), jv2(1));

				cv::circle(_3d,pj,6,cv::Scalar(50,200,250),-1);
			}
		}

		cv::imshow("rgb", tex);
		cv::imshow("3d", _3d);
		char in = cv::waitKey(10);

		switch(in){
		case 'q':
			return 0;
		case 'z':
			--frame;
			while(vidRecord[frame].videoFrame.mat.empty()){
				--frame;
			}
			if(frame < 0) frame = 0;
			break;
		case 'x':
			++frame;
			while(vidRecord[frame].videoFrame.mat.empty()){
				++frame;
			}
			if(frame >= vidRecord.size()) frame = vidRecord.size()-1;
			break;
		case 'r':
			angle_y = 0;
			angle_x = 0;
			break;
		case 'c':
			alljoints = !alljoints;
			break;
		case 's':
			
			for(int i=0;i<vidRecord.size();++i){
				vidRecord[i].kinectPoints = wcSkeletons[i];

				//if(!vidRecord[i].cam2World.empty())
				//{
				//	vidRecord[i].kinectPoints.points = vidRecord[i].cam2World.inv() * vidRecord[i].kinectPoints.points;
				//}
			}

			SaveVideo(&vidRecord, getCameraMatrixScene(),  "map000000_aoto2_edit/video/");
			break;
		}
	}
}

cv::Point start,prev;
bool mousedn = false;
bool rmousedn = false;
int cjoint;

void onMouse(int e, int x, int y, int, void *){
	switch (e)
	{
	case cv::EVENT_LBUTTONDOWN:
		start.x = x;
		start.y = y;
		prev = start;
		mousedn = true;
		rmousedn = false;
		break;
	case cv::EVENT_LBUTTONUP:
		mousedn = false;
		rmousedn = false;
		break;
	case cv::EVENT_MOUSEMOVE:
		if(mousedn){
			cv::Point curr(x,y);
			angle_y += (curr.x - prev.x)/180.*CV_PI;
			angle_x += (curr.y - prev.y)/180.*CV_PI;
			prev = curr;
		}
		else if(rmousedn){
			if(!alljoints){
				if(cjoint != -1){
					cv::Vec3f ray = lerpPoint(x,y,boundingBoxLerp,lc);
					cv::Vec3f newPos = vectorProject(mat_to_vec3(trans * wcSkeletons[frame].points.col(cjoint)), ray);
					cv::Mat(trans.inv() * vec3_to_mat4(newPos)).copyTo(wcSkeletons[frame].points.col(cjoint));
					calculateSkeletonOffsetPoints(vidRecord, wcSkeletons, cylinderBody);
				}
			}else{
				
				if(cjoint != -1){
					cv::Vec3f ray = lerpPoint(x,y,boundingBoxLerp,lc);
					cv::Vec3f newPosProj = vectorProject(mat_to_vec3(trans * wcSkeletons[frame].points.col(cjoint)), ray);
					cv::Mat newPos = trans.inv() * vec3_to_mat4(newPosProj);
					
					cv::Mat oldPos = wcSkeletons[frame].points.col(cjoint);

					cv::Mat diff = newPos - oldPos;

					newPos.copyTo(wcSkeletons[frame].points.col(cjoint));

					for(int joint = 0; joint < NUMJOINTS; ++joint){
						if(joint != cjoint){

							cv::Mat newPosCopyTransform = wcSkeletons[frame].points.col(joint) + diff;

							newPosCopyTransform.copyTo(wcSkeletons[frame].points.col(joint));
						}
					}

					calculateSkeletonOffsetPoints(vidRecord, wcSkeletons, cylinderBody);
				}
			}
		}
		break;
	case cv::EVENT_RBUTTONDOWN:
		{
			mousedn = false;
			rmousedn = true;

			cv::Point curr(x,y);

			float cdist=10000;
			cjoint=-1;

			for(int joint=0;joint<NUMJOINTS;++joint)
			{
				cv::Vec3f jv = mat_to_vec3(trans * wcSkeletons[frame].points.col(joint));
				cv::Vec2f jv2 = mat4_to_vec2(getCameraMatrixScene() * vec3_to_mat4(jv));
				cv::Point pj(jv2(0), jv2(1));

				float dist = cv::norm(curr-pj);
				if(dist < cdist && dist < 10){
					cdist = dist;
					cjoint = joint;
				}
			}
			break;
		}
	case cv::EVENT_RBUTTONUP:
		{
			mousedn = false;
			rmousedn = false;
			cjoint = -1;
		}
	default:
		break;
	}
}