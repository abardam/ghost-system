#pragma once
#include <opencv2\opencv.hpp>


cv::Mat depth2BGR(cv::Mat depthMat);


//3-vec to 4x1 homogenous matrix
cv::Mat vec3_to_mat4(cv::Vec3f vec);

//3x3 matrix to 4x4 homogenous matrix
cv::Mat mat3_to_mat4(cv::Mat mat);

//4x1 or 3x1 mat to 3-vec
cv::Vec3f mat_to_vec3(cv::Mat mat);


cv::Vec3f vec4_to_vec3(cv::Vec4f v);
cv::Vec2f vec3_to_vec2(cv::Vec3f v);

//adds a row to a mat, changes leftmost element to 1
cv::Mat mat_to_homo(cv::Mat mat);

//changes homogenous mat to a 2-vec
cv::Vec2f mat4_to_vec2(cv::Mat mat);


//3-vec to 4x4 transformation matrix
cv::Mat getTranslationMatrix(cv::Vec3f tvec);

cv::Mat getRotationMatrix(cv::Vec2f vector);
cv::Mat getRotationMatrix(float angle);
cv::Mat getTransformMatrix(cv::Vec2f a, cv::Vec2f b);
cv::Mat getTransformMatrix_r(cv::Vec2f a, cv::Vec2f b);
cv::Mat getScaleMatrix(float x, float y, float z);

cv::Mat getRotationMatrix(cv::Vec3f vector, cv::Vec3f baseVector = cv::Vec3f(0,0,1));


//axis-angle. returns a 3x3 mat
cv::Mat getRotationMatrix(cv::Vec3f axis, float angle);

//axis-angle. returns a 4x4 mat
cv::Mat getRotationMatrix4(cv::Vec3f axis, float angle);

cv::Mat calcTransform_r(cv::Vec3f a, cv::Vec3f b);
cv::Mat calcTransform(cv::Vec3f a, cv::Vec3f b);

cv::Vec3f vectorProject(cv::Vec3f A, cv::Vec3f B, cv::Vec3f Bhat = cv::Vec3f(0,0,0));

//assuming: from an origin (0,0,0), to main point (0,0,1), with direction vector (0,0,0)-(0,-1,0),
//get transformation matrix to points tarA (new origin), tarB (new main point), with facing toward the screen
cv::Mat getCylinderTransform(cv::Vec3f tarA, cv::Vec3f tarB, float partRadius, float leftOffset, float rightOffset);

float angleBetweenTwoVectors(cv::Vec3f A, cv::Vec3f B);

cv::Vec3f getBestFacingVector(cv::Vec3f A, cv::Vec3f B);


bool calculateIntersection(cv::Vec2f a1, cv::Vec2f b1, cv::Vec2f a2, cv::Vec2f b2, float * lambda1, float * lambda2);

//4x4 homogeneous camera matrix is singular because of the 0; need to convert to 3x3 first. returns a 3x3 matrix.
cv::Mat invertCameraMatrix(cv::Mat cameraMatrix);

//a -> (0,0,0) , b-> (0,0,1)
//4x4 transformation matrix
cv::Mat segmentZeroTransformation(cv::Vec3f a, cv::Vec3f b, cv::Mat * inverse = 0);

//a1 -> a2, b1 -> b2
//4x4 transformation matrix
cv::Mat segmentTransformation(cv::Vec3f a1, cv::Vec3f b1, cv::Vec3f a2, cv::Vec3f b2);

//new cylinder facing
//to get the actual 3d point, add it to point a
//facing 0 will give a facing into the screen (e.g. given vertical cylinder, [0;0;1] )
cv::Vec3f cylinderFacingVector(cv::Vec3f a, cv::Vec3f b, float facing);

//precomputed version
cv::Vec3f cylinderFacingVector(cv::Vec3f axis, cv::Mat facingVec, float facing);

//helper for cylidnerFacingVector(precomputed)
std::pair<cv::Vec3f, cv::Mat> cylinderFacingHelper(cv::Vec3f a, cv::Vec3f b);

float sqrSum(cv::Mat m);