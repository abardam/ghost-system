#include <opencv2\opencv.hpp>
#include "cvutil.h"
#include "definitions.h"

float calculateScore(cv::Mat a, cv::Mat b){
	float bst=0;	
	for(int j=0; j<NUMJOINTS; ++j){
		bst += cv::norm(b.col(j)-a.col(j));
	}

	return bst;
}

cv::Mat depth2BGR(cv::Mat depthMat){
	cv::Mat ret(480, 640, CV_8UC3);

	for(int r=0;r<480;++r){
		for(int c=0;c<640;++c){
			
			int dval = 256 - depthMat.at<unsigned short>(r,c) * (256.0/8000);

			ret.at<cv::Vec3b>(r,c) = cv::Vec3b(dval, dval, dval);
		}
	}

	return ret;
};

cv::Mat vec3_to_mat4(cv::Vec3f vec){
	cv::Mat retval;
	cv::vconcat(cv::Mat(vec), cv::Mat::ones(1,1,cv::DataType<float>::type), retval);
	return retval;
}
cv::Mat mat3_to_mat4(cv::Mat mat){

	if(mat.size().width == 3)
		cv::hconcat(mat, cv::Mat::zeros(3,1,mat.type()), mat);

	cv::vconcat(mat, cv::Mat::zeros(1,4,cv::DataType<float>::type), mat);
	mat.at<float>(3,3) = 1;
	return mat;
}
cv::Vec3f mat_to_vec(cv::Mat mat){
	float w;
	if(mat.total() == 4) {
		w = mat.at<float>(3);
		if(w == 0) w = 1;
	}
	else w = 1;

	return cv::Vec3f(mat.at<float>(0)/w, mat.at<float>(1)/w, mat.at<float>(2)/w);
}
cv::Vec3f mat_to_vec3(cv::Mat mat){
	float w;
	if(mat.total() == 4) {
		w = mat.at<float>(3);
		if(w == 0) w = 1;
	}
	else w = 1;
	return cv::Vec3f(mat.at<float>(0)/w, mat.at<float>(1)/w, mat.at<float>(2)/w);
}

cv::Mat getTranslationMatrix(cv::Vec3f tvec){
	float transl[] = 
		{1,0,0,tvec(0), 
		0,1,0,tvec(1), 
		0,0,1,tvec(2), 
		0,0,0,1};

	return cv::Mat(4,4,cv::DataType<float>::type, transl).clone();
}



cv::Mat mat_to_homo(cv::Mat mat){
	cv::Mat temp;
	cv::vconcat(mat, cv::Mat::zeros(1,mat.cols,mat.type()), temp);
	temp.at<float>(temp.rows-1,temp.cols-1) = 1;
	return temp;
};

cv::Vec2f mat4_to_vec2(cv::Mat mat){
	return cv::Vec2f(mat.at<float>(0)/mat.at<float>(2),mat.at<float>(1)/mat.at<float>(2));
};


cv::Vec3f vec4_to_vec3(cv::Vec4f v){
	return cv::Vec3f(v[0]/v[3], v[1]/v[3], v[2]/v[3]);
}

cv::Vec2f vec3_to_vec2(cv::Vec3f v){
	return cv::Vec2f(v[0]/v[2], v[1]/v[2]);
}

cv::Mat getRotationMatrix(cv::Vec2f vector){
	float theta = atan2(vector[1], vector[0]);
	float rmatdata[] = {cos(theta), -sin(theta), sin(theta), cos(theta)};
	return cv::Mat(2,2,cv::DataType<float>::type,rmatdata).clone();
};

//radians
cv::Mat getRotationMatrix(float theta){
	float rmatdata[] = {cos(theta), -sin(theta), sin(theta), cos(theta)};
	return cv::Mat(2,2,cv::DataType<float>::type,rmatdata).clone();
}


cv::Mat getScaleMatrix(float x, float y, float z){
	cv::Mat a = cv::Mat::eye(4,4,cv::DataType<float>::type);
	a.at<float>(0,0) = x;
	a.at<float>(1,1) = y;
	a.at<float>(2,2) = z;
	return a;
}

cv::Mat getTransformMatrix(cv::Vec2f a, cv::Vec2f b){
	cv::Vec2f translation = -a;
	cv::Vec2f base = b - a;
	cv::Mat rotation = getRotationMatrix(base);
	
	cv::Mat rotation_r;
	cv::transpose(rotation, rotation_r);

	cv::Mat temp_translation;
	cv::hconcat(cv::Mat::eye(3,2,cv::DataType<float>::type), mat_to_homo(cv::Mat(translation)), temp_translation);
	

	cv::Mat temp_rotation;
	cv::hconcat(rotation_r, cv::Mat::zeros(2,1,cv::DataType<float>::type), temp_rotation);

	return mat_to_homo(temp_rotation)*temp_translation;
};

cv::Mat getTransformMatrix_r(cv::Vec2f a, cv::Vec2f b){
	cv::Vec2f translation = a;
	cv::Vec2f base = b - a;
	cv::Mat rotation = getRotationMatrix(base);
	

	cv::Mat temp_translation;
	cv::hconcat(cv::Mat::eye(3,2,cv::DataType<float>::type), mat_to_homo(cv::Mat(translation)), temp_translation);
	

	cv::Mat temp_rotation;
	cv::hconcat(rotation, cv::Mat::zeros(2,1,cv::DataType<float>::type), temp_rotation);

	return temp_translation*mat_to_homo(temp_rotation);
};

//drawing


void lineAt(cv::Mat img, cv::Vec2f a, cv::Vec2f b, IMGPIXEL color){

	//clamp a and b to the edge of the img
	float lambda;
	for(int i=0;i<=1;++i){
		lambda = 0;
		if(a[i] < 0){
			lambda = (0 - a[i])/(b[i]-a[i]);
		}else if(a[i] > img.size[1-i]-1){
			lambda = (img.size[1-i]-1-a[i])/(b[i]-a[i]);
		}
		a = a + lambda*(b-a);
		
		lambda = 0;
		if(b[i] < 0){
			lambda = (0 - b[i])/(a[i]-b[i]);
		}else if(b[i] > img.size[1-i]-1){
			lambda = (img.size[1-i]-1-b[i])/(a[i]-b[i]);
		}
		b = b + lambda*(a-b);
	}

	int x0 = a[0];
	int x1 = b[0];
	int y0 = a[1];
	int y1 = b[1];
	int dx = abs(x1-x0);
	int dy = abs(y1-y0);
	int sx, sy;
	if(x0 < x1) sx = 1; else sx = -1;
	if(y0 < y1) sy = 1; else sy = -1;
	int err = dx-dy;
 
	int e2;

	while(true)
	{
		if(y0 < 0 || y0 >= img.rows ||
			x0 < 0 || x0 >= img.cols) return;
		img.at<IMGPIXEL>(y0,x0) = color;
		if(x0 == x1 && y0 == y1) return;
		e2 = 2*err;
		if(e2 > -dy)
		{
			err = err - dy;
			x0 = x0 + sx;
		}
		if(x0 == x1 && y0 == y1)
		{
			img.at<IMGPIXEL>(y0,x0) = color;
			return;
		}
		if(e2 < dx){
			err = err + dx;
			y0 = y0 + sy;
		}
	}
};


//rotations



cv::Mat getRotationMatrix(cv::Vec3f vector, cv::Vec3f baseVector){
	vector = cv::normalize(vector);
	baseVector = cv::normalize(baseVector);

	cv::Vec3f cross = cv::normalize(baseVector.cross(vector));
	float angle = acos(baseVector.dot(vector));

	float rotdata[] = { cos(angle) + pow(cross(0),2)*(1-cos(angle)), cross(0)*cross(1)*(1-cos(angle))-cross(2)*sin(angle), cross(0)*cross(2)*(1-cos(angle))+cross(1)*sin(angle),
		cross(1)*cross(0)*(1-cos(angle))+cross(2)*sin(angle), cos(angle)+pow(cross(1),2)*(1-cos(angle)), cross(1)*cross(2)*(1-cos(angle))-cross(0)*sin(angle),
		cross(2)*cross(0)*(1-cos(angle))-cross(1)*sin(angle), cross(2)*cross(1)*(1-cos(angle))+cross(0)*sin(angle), cos(angle)+pow(cross(2),2)*(1-cos(angle)) };

	return cv::Mat(3,3,cv::DataType<float>::type,rotdata).clone();

};

cv::Mat calcTransform(cv::Vec3f a, cv::Vec3f b){
	cv::Vec3f translation = -a;

	cv::Vec3f b2 = b + translation;
	float scale = cv::norm(b2);
	cv::Mat rotMat = getRotationMatrix(b2);
	//rotMat = cv::Mat::eye(3,3,rotMat.type());
	cv::Mat invRotMat;
	cv::transpose(rotMat, invRotMat);

	cv::Mat scaleMat = cv::Mat::eye(3,3,rotMat.type()) / scale;
	invRotMat = invRotMat * (scaleMat);

	cv::Mat transformMat;

	//translation = cv::Vec3f(0,0,0);

	cv::hconcat(invRotMat, invRotMat * cv::Mat(translation), transformMat);
	transformMat = mat3_to_mat4(transformMat);

	return transformMat;
};


cv::Mat calcTransform_r(cv::Vec3f a, cv::Vec3f b){
	cv::Vec3f translation_r = a;
	cv::Mat transformMat_r;
	cv::Vec3f translation = -a;
	cv::Vec3f b2 = b + translation;
	float scale = cv::norm(b2);
	
	cv::Mat rotMat = getRotationMatrix(b2);
	//rotMat = cv::Mat::eye(3,3,rotMat.type());

	rotMat = rotMat * (cv::Mat::eye(3,3,rotMat.type()) * scale);
	
	//translation_r = cv::Vec3f(0,0,0);
	cv::hconcat(rotMat, cv::Mat(translation_r), transformMat_r);
	transformMat_r = mat3_to_mat4(transformMat_r);

	return transformMat_r;
}

cv::Mat getRotationMatrix(cv::Vec3f axis, float t){
	axis = cv::normalize(axis);

	float l = axis[0];
	float m = axis[1];
	float n = axis[2];

	float ct = cos(t);
	float st = sin(t);

	float matdata[] = {l*l*(1-ct)+ct, m*l*(1-ct)-n*st, n*l*(1-ct)+m*st,
		l*m*(1-ct)+n*st, m*m*(1-ct)+ct, n*m*(1-ct)-l*st,
		l*n*(1-ct)-m*st, m*n*(1-ct)+l*st, n*n*(1-ct)+ct};

	return cv::Mat(3,3,cv::DataType<float>::type, matdata).clone();
}

//project vector A onto vector B
cv::Vec3f vectorProject(cv::Vec3f A, cv::Vec3f B){
	cv::Vec3f Bhat = cv::normalize(B);

	return A.dot(Bhat) * Bhat;
}

float angleBetweenTwoVectors(cv::Vec3f A, cv::Vec3f B){
	return acos(A.dot(B)/(cv::norm(A)*cv::norm(B)));
}

//

//cylinder rotation

cv::Vec3f getBestFacingVector(cv::Vec3f A, cv::Vec3f B){
	
	cv::Vec3f C = /*cv::normalize(A)*/-A;
	B -= A;
	A -= A;

	cv::Vec3f AC = C-A;
	cv::Vec3f AB = B-A;

	cv::Vec3f AC_prime = vectorProject(AC, AB);
	cv::Vec3f C_primeC = C-AC_prime;

	return C_primeC;
}
cv::Mat getCylinderRotateTransform(cv::Vec3f tarA, cv::Vec3f tarB, cv::Vec3f newA2, float ratio){
	cv::Vec3f targetVec = getBestFacingVector(tarA, tarB);

	return getRotationMatrix( targetVec, newA2);
}


//assuming: from an origin (0,0,0), to main point (0,0,1), with direction vector (0,0,0)-(0,-1,0),
//get transformation matrix to points tarA (new origin), tarB (new main point), with facing toward the screen
cv::Mat getCylinderTransform(cv::Vec3f tarA_, cv::Vec3f tarB_, float partRadius, float leftOffset, float rightOffset){
#if GH_MODELFITTING == GH_MF_OLD
	cv::Vec3f tarA = tarA_ + (tarA_ - tarB_) * -leftOffset;
	cv::Vec3f tarB = tarB_ + (tarB_ - tarA_) * rightOffset;
#elif GH_MODELFITTING == GH_MF_CYLPROJ
	cv::Vec3f tarA = tarB_ + (tarA_ - tarB_) * leftOffset;
	cv::Vec3f tarB = tarA_ + (tarB_ - tarA_) * rightOffset;
#endif
	
	cv::Vec4f a2(0,-1,0,1);
	cv::Mat transmat = calcTransform_r(tarA, tarB);
	
	cv::Mat transform1 =  getTranslationMatrix(-tarA) *  transmat;

	cv::Vec3f newa2 =  mat_to_vec(transform1 * cv::Mat(a2));

#if GH_MODELFITTING == GH_MF_OLD
	return getTranslationMatrix(tarA) * mat3_to_mat4(getCylinderRotateTransform(tarA, tarB, newa2,1) ) * transform1 *getScaleMatrix(partRadius, partRadius, 1);
#elif GH_MODELFITTING == GH_MF_CYLPROJ
	partRadius /= cv::norm(tarA - tarB);

	return getTranslationMatrix(tarA) * mat3_to_mat4(getCylinderRotateTransform(tarA, tarB, newa2,1) ) * transform1 * getScaleMatrix(partRadius, partRadius, 1);
#endif

}


int calcBinFromFacing(cv::Vec3f facing){
	//first project facing to the horizontal (a=0, b=1, c=0, d=0) plane

	cv::Vec3f vert(0,1,0);

	float fdot = facing.dot(vert);
	cv::Vec3f fproj = fdot * vert;

	cv::Vec3f frej = facing - fproj;

	//frej should have y component of 0 now...

	float angle = atan2f(frej(2), frej(0));

	angle += CV_PI;
	//angle now goes from 0 to 2pi
	//bin it

	float x = NUMBINS/(2*CV_PI);

	int bin = angle * x;

	return bin;
}

//return false if parallel
bool calculateIntersection(cv::Vec2f a1, cv::Vec2f b1, cv::Vec2f a2, cv::Vec2f b2, float * lambda1, float * lambda2){
	cv::Vec2f A = a1 - a2;
	cv::Vec2f B = b1 - a1; 
	cv::Vec2f D = b2 - a2;

	float DB = D(0)*B(1)-D(1)*B(0);
	if(DB == 0) return false;

	*lambda1 = (D(1)*A(0)-D(0)*A(1))/DB;
	
	if(D(0) != 0){
		*lambda2 = (A(0)+B(0)**lambda1)/D(0);
	}else{
		*lambda2 = (A(1)+B(1)**lambda1)/D(1);
	}

	return true;
}


cv::Mat invertCameraMatrix(cv::Mat cameraMatrix){
	cv::Mat cam33 = cameraMatrix.colRange(0,3).rowRange(0,3);
	cv::Mat invcam33 = cam33.inv();
	return (invcam33);
}

cv::Mat segmentZeroTransformation(cv::Vec3f cyl_a, cv::Vec3f cyl_b){

	//transform the space:
	cv::Vec3f axis_1 = cyl_b - cyl_a;
	cv::Vec3f axis_2 = cv::Vec3f(0,0,1);

	cv::Vec3f axis = axis_1.cross(axis_2);
	float angle = acos(axis_1.dot(axis_2)/(cv::norm(axis_1)));


	cv::Mat transformation = mat3_to_mat4(getRotationMatrix(axis,angle)) * getTranslationMatrix(-cyl_a);

	return transformation;
}

//stretches along axis, doesnt scale uniformly
cv::Mat segmentTransformation(cv::Vec3f a1, cv::Vec3f b1, cv::Vec3f a2, cv::Vec3f b2){
	//transform the space:
	cv::Vec3f axis_1 = b1 - a1;
	cv::Vec3f axis_2 = b2 - a2;

	float scaleRatio = cv::norm(axis_2)/cv::norm(axis_1);

	cv::Vec3f zero(0,0,1);

	cv::Vec3f axis_1z = axis_1.cross(zero);
	float angle_1z = acos(axis_1.dot(zero)/(cv::norm(axis_1)));

	cv::Vec3f axis_z2 = zero.cross(axis_2);
	float angle_z2 = acos(zero.dot(axis_2)/(cv::norm(axis_2)));

	cv::Mat transformation = getTranslationMatrix(a2) * mat3_to_mat4( getRotationMatrix(axis_z2, angle_z2) ) * getScaleMatrix(1,1,scaleRatio) * mat3_to_mat4(getRotationMatrix(axis_1z,angle_1z)) * getTranslationMatrix(-a1);

	return transformation;
}

//new cylinder facing
//to get the actual 3d point, add it to point a
cv::Vec3f cylinderFacingVector(cv::Vec3f a, cv::Vec3f b, float facing){
	cv::Vec3f axis = b-a;
	cv::Vec3f perpVec = a.cross(axis);
	cv::Vec3f facingVec = cv::normalize(axis.cross(perpVec));
	return mat_to_vec(getRotationMatrix(axis, facing) * cv::Mat(facingVec));
}

float sqrSum(cv::Mat m){
	float ret = 0;
	for(int j=0;j<m.rows;++j){
		for(int k=0;k<m.cols;++k){
			ret += m.at<float>(j,k) * m.at<float>(j,k);
		}
	}
	return ret;
}