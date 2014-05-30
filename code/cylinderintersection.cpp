#include "cylinderintersection.h"
#include "cvutil.h"

#include <opencv2\opencv.hpp>

cv::Vec3f raycast(cv::Vec2f pt, cv::Mat invCameraMatrix){
	cv::Vec3f pt3(pt(0), pt(1), 1);
	cv::Vec3f repro = mat_to_vec(invCameraMatrix * cv::Mat(pt3));
	return cv::normalize(repro);
}

//ray from the origin to plane defined by pt and vec
int rayPlaneIntersection(cv::Vec3f ray, cv::Vec3f plane_Pt, cv::Vec3f plane_Vec, cv::Vec3f * out){
	cv::Vec3f u = ray;
	cv::Vec3f w = -plane_Pt;

	float D = plane_Vec.dot(u);
	float N = -plane_Vec.dot(w);

	if(abs(D) < 0.0001){
		if(N==0)
			return 2;
		else 
			return 0;
	}

	float sI = N/D;
	if(sI < 0) return 0;

	*out = sI * u;
	return 1;
}


//dont use this
int rayCylinderIntersection(cv::Vec3f ray, cv::Vec3f cyl_a, cv::Vec3f cyl_b, float radius, cv::Vec3f * out){
	cv::Vec3f cap_a, cap_b;
	int res_a = rayPlaneIntersection(ray, cyl_a, cyl_b-cyl_a, &cap_a);
	int res_b = rayPlaneIntersection(ray, cyl_b, cyl_a-cyl_b, &cap_b);

	bool retvalid = false;


	//quadratic formula: line equation x = (y-a_2)/b_2 = (z-a_3)/b_3
	//cylinder equation r^2 = (x-c_1)^2 + (y-c_2)^2

	if(res_a == 1){
		float dist = cv::norm(cap_a - cyl_a);
		if(dist <= radius){
			if(!retvalid || (*out)(2) > cap_a(2)){
				(*out) = cap_a;
				retvalid = true;
			}
		}
	}
	
	if(res_b == 1){
		float dist = cv::norm(cap_b - cyl_b);
		if(dist <= radius){
			if(!retvalid || (*out)(2) > cap_b(2)){
				(*out) = cap_b;
				retvalid = true;
			}
		}
	}

	if(retvalid) return 1;
	else return 0;
}

//from an old project
//the idea is to transform the space s.t. the cylinder sits at the origin with its axis on the z-axis
//origin: transformed origin, ray: transformed ray, radius and height: cylinder stuff
std::vector<std::pair<float,cv::Vec3f>> rayCylinderIntersectionPoints(cv::Vec3f origin, cv::Vec3f ray, float radius, float height){
	//ray = origin + (ray-origin) * -100;
	//ray = ray / cv::norm(ray);

	ray = cv::normalize(ray - origin);

	float a = ray(0)*ray(0) + ray(1)*ray(1);
	float b = 2*origin(0)*ray(0)+2*origin(1)*ray(1);
	float c = origin(0)*origin(0) + origin(1)*origin(1) -radius*radius;


	float plus = (-b + sqrt(b*b-4*a*c))/(2*a);
	float minus = (-b - sqrt(b*b-4*a*c))/(2*a);

	cv::Vec3f vplus(origin(0)+plus*ray(0), origin(1)+plus*ray(1), origin(2)+plus*ray(2));
	cv::Vec3f vminus(origin(0)+minus*ray(0), origin(1)+minus*ray(1), origin(2)+minus*ray(2));
	
	std::vector<std::pair<float,cv::Vec3f>> ret;

	if(vplus(2) > 0 && vplus(2) < height)
	{
		ret.push_back(std::pair<float,cv::Vec3f>(plus,vplus));
	}

	if(vminus(2) > 0 && vminus(2) < height)
	{
		ret.push_back(std::pair<float,cv::Vec3f>(minus,vminus));
	}

	if( (vplus(2) <= 0 && vminus(2) > 0) || (vminus(2) <= 0 && vplus(2) > 0)){
		float extra = -origin(2)/ray(2);
		cv::Vec3f vextra(origin(0)+extra*ray(0), origin(1)+extra*ray(1), origin(2)+extra*ray(2));
		ret.push_back(std::pair<float,cv::Vec3f>(extra,vextra));
	}

	if( (vplus(2) >= height && vminus(2) < height) || (vminus(2) >= height && vplus(2) < height)){
		float extra = (height-origin(2))/ray(2);
		cv::Vec3f vextra(origin(0)+extra*ray(0), origin(1)+extra*ray(1), origin(2)+extra*ray(2));
		ret.push_back(std::pair<float,cv::Vec3f>(extra,vextra));
	}

	return ret;
}

int rayCylinder(cv::Vec3f ray, cv::Vec3f cyl_a, cv::Vec3f cyl_b, float radius, cv::Vec3f * out){

	
	
	cv::Vec3f cyl_axis = cyl_b - cyl_a;
	cv::Mat transformation = segmentZeroTransformation(cyl_a, cyl_b);

	//std::cout << "axis: " << t_b-t_a << std::endl;

	//cv::Vec3f origin = mat_to_vec(transformation * cv::Mat(cv::Vec4f(0,0,0,1)));
	//cv::Vec3f ray_trans = mat_to_vec(transformation * vec3_to_mat4(ray));
	float height = cv::norm(cyl_axis);

	return rayCylinder2(transformation, ray, radius, height, out);

	//std::vector<std::pair<float,cv::Vec3f>> ret = rayCylinderIntersectionPoints(origin, ray_trans, radius, height);
	//
	//if(ret.size() == 0){
	//	return 0;
	//}else{
	//	if(ret[0].first < ret[1].first){
	//		*out = mat_to_vec(transformation.inv()*vec3_to_mat4(ret[0].second));
	//	}else{
	//		*out = mat_to_vec(transformation.inv()*vec3_to_mat4(ret[1].second));
	//	}
	//	return 1;
	//}
}



//should be a copy of rayCylinder but with all transformations being done at the start so its more efficient
int rayCylinder2(cv::Mat transformation, cv::Vec3f ray, float radius, float height, cv::Vec3f * out){
	
	cv::Vec3f origin = mat_to_vec(transformation * cv::Mat(cv::Vec4f(0,0,0,1)));
	cv::Vec3f ray_trans = mat_to_vec(transformation * vec3_to_mat4(ray));

	std::vector<std::pair<float,cv::Vec3f>> ret = rayCylinderIntersectionPoints(origin, ray_trans, radius, height);

	if(ret.size() == 0){
		return 0;
	}else{
		if(ret[0].first < ret[1].first){
			*out = mat_to_vec(transformation.inv()*vec3_to_mat4(ret[0].second));
		}else{
			*out = mat_to_vec(transformation.inv()*vec3_to_mat4(ret[1].second));
		}
		return 1;
	}
}