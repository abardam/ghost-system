#include "ghostcam.h"
#include "cvutil.h"
#include "camlerp.h"

#define CAPTURE_SIZE_X 640
#define CAPTURE_SIZE_Y 480

cv::Mat cameraMatrix;
cv::Mat invCameraMatrix;

void setCameraMatrix(float param0, float param1, float param2, float param3, float width, float height)
{
	cameraMatrix = cv::Mat(3,4,cv::DataType<float>::type,cv::Scalar(0));

	cameraMatrix.at<float>(0,0) = param0 * width;
	cameraMatrix.at<float>(1,1) = - param1 * height;
	cameraMatrix.at<float>(0,2) = param2 * width - 0.5					-5;	//empirically, the - and + 5 looks closer to the original toScreen matrix.
	cameraMatrix.at<float>(1,2) = - (param3 * height - 0.5) + height	+5;
	cameraMatrix.at<float>(2,2) = 1;

	//std::cout << "cam matrix: \n" << cameraMatrix << std::endl;

}

void setCameraMatrix(cv::Mat mat){
	cameraMatrix = mat.rowRange(0,3).colRange(0,4).clone();
	invCameraMatrix = invertCameraMatrix(cameraMatrix);
	initLerp(CAPTURE_SIZE_X, CAPTURE_SIZE_Y);
}

/*
void calculateCameraMatrix(){
	//inverse camera matrix from toScreen

	int numPts = 64;
	
	cv::Mat m(4,numPts,cv::DataType<float>::type,cv::Scalar(1));
	cv::Mat m2 = cv::Mat(3,numPts,cv::DataType<float>::type,cv::Scalar(1));
	std::vector<cv::Vec3f> wldPts;
	std::vector<cv::Vec2f> scrnPts;

	for(int i=0;i<numPts;++i){

		int x = 2 - (i/4)%4;
		int y = 2 - i/16;
		int z = i%4+1;

		
		wldPts.push_back(cv::Vec3f(x,y,z));

		for(int j=0;j<3;++j)
			m.at<float>(j,i) = wldPts[i](j);

		scrnPts.push_back(KINECT::toScreen(wldPts[i]));
		for(int j=0;j<3;++j){
			if(j<2){
				m2.at<float>(j,i) = scrnPts[i](j);
			}
		}
	}

	cv::Mat minv;
	double det = cv::invert(m, minv,CV_SVD);

	std::cout << "determinant: " << det << std::endl;

	cameraMatrix = m2 * minv;

	std::cout << "inv m:\n" << minv << "\nm2:\n" << m2 << std::endl;
	
	std::cout << "cam matrix: \n" << cameraMatrix << std::endl;
}*/


cv::Mat getCameraMatrix(){
	return cameraMatrix;
}

cv::Mat getInvCameraMatrix(){
	return invCameraMatrix;
}

cv::Vec2f toScreen(cv::Vec3f v){
#if INIT_KINECT
	return KINECT::toScreen(v);
#else
	cv::Mat _v = vec3_to_mat4(v);
	cv::Mat __v = getCameraMatrix() * _v;
	return mat4_to_vec2(__v);
#endif
};

