#include "bodybuild.h"
#include "definitions.h"
#include "CylinderBody.h"
#include "loader.h"
#include "cvutil.h"
#include "ghostcam.h"

//from KinectManager.h
#define CAPTURE_SIZE_X	640
#define CAPTURE_SIZE_Y	480

typedef std::vector<cv::RotatedRect> Config;

static bool cmpBG(cv::Vec3b pix){
	return pix == cv::Vec3b(255,255,255);
}
static bool cmpFG(cv::Vec3b pix){
	return pix != cv::Vec3b(255,255,255);
}

//counts pixels that fit into the category given by cmpfnc
static int countPixels(cv::Mat im, cv::RotatedRect rec, bool (*cmpFnc)(cv::Vec3b)){

	std::vector<cv::Point2f> pts(4);
	rec.points(pts.data());

	//1st rotate the image accdg to the rec
	//we try to resize the img s.t. the rotated area is all that remains

	cv::Mat rotate = cv::getRotationMatrix2D(rec.center, rec.angle, 1);
	//std::cout << "rotate: " << rotate << std::endl;
	//bring it back
	rotate.at<double>(0,2) -= (rec.center.x - rec.size.width/2);
	rotate.at<double>(1,2) -= (rec.center.y - rec.size.height/2);
	//std::cout << "rotate: " << rotate << std::endl << "-------------\n";

	cv::Rect brect = cv::boundingRect(pts);

	cv::Mat rotim;
	cv::warpAffine(im, rotim, rotate, rec.size, 1, cv::BORDER_CONSTANT, cv::Scalar(255,255,255));

	//cv::imshow("riot", rotim);

	int cnt = 0;
	for(int r=0; r<floor(rec.size.height);++r){
		for(int c=0; c<floor(rec.size.width); ++c){
			if((*cmpFnc)(rotim.at<cv::Vec3b>(cv::Point(c,r))))
				++cnt;
		}
	}
	return cnt;
}

// (alpha)(f-b)^2 - A
static float cost1(cv::Mat im, cv::RotatedRect rec, float idealmult){
	float alpha = 10;
	cv::RotatedRect ideal = rec;
	ideal.size.height = ideal.size.width*idealmult;
	int idealF = countPixels(im, ideal, cmpFG);

	int f = countPixels(im, rec, cmpFG);
	int b = countPixels(im, rec, cmpBG);

	int f_not = idealF - f;



	return alpha*std::pow((float)f_not - b, 2);// - rec.size.area();

}


// (f-b)^2/A
static float cost2(cv::Mat im, cv::RotatedRect rec, float idealmult){
	cv::RotatedRect ideal = rec;
	ideal.size.height = ideal.size.width*idealmult;
	int idealF = countPixels(im, ideal, cmpFG);

	int f = countPixels(im, rec, cmpFG);
	int b = countPixels(im, rec, cmpBG);

	int f_not = idealF - f;



	return std::pow((float)f_not - b,2);// / rec.size.area();

}

std::vector<BodyPartParam> rectFitting(Skeleton skeletonPositions, CroppedCvMat im, CylinderBody * cylinderBody){
	std::vector<BodyPartParam> ret(NUMLIMBS);

	cv::Mat bestim = im.mat;
	//initialize rectangles
	Config config(NUMLIMBS);

	int frame = cylinderBody->bestFrame;
	cv::Vec2f voff = cv::Vec2f(im.offset.x, im.offset.y);
	
	//hardcoded body part lengths
	std::vector<float> bpLengthsLeft(NUMLIMBS, 1);
	std::vector<float> bpLengthsRight(NUMLIMBS, 1);
	
	bpLengthsLeft[HEAD] = 1.5;
	bpLengthsLeft[CHEST] = 1.25;

	bpLengthsRight[LOWERARM_LEFT] = 1.25;
	bpLengthsRight[LOWERARM_RIGHT] = 1.25;
	
	bpLengthsRight[LOWERLEG_LEFT] = 1.25;
	bpLengthsRight[LOWERLEG_RIGHT] = 1.25;


	for(int i=0;i<NUMLIMBS;++i){
		int p1 = getLimbmap()[i].first;
		int p2 = getLimbmap()[i].second;

		cv::Vec3f v1 = mat_to_vec3(skeletonPositions.points.col(p1));
		cv::Vec3f v2 = mat_to_vec3(skeletonPositions.points.col(p2));

		//hardcoded body part lengths
		cv::Vec3f lv1 = v2 + (v1 - v2)*bpLengthsLeft[i];
		cv::Vec3f lv2 = v1 + (v2 - v1)*bpLengthsRight[i];

		cv::Vec2f s1 = toScreen(lv1) - voff;
		cv::Vec2f s2 = toScreen(lv2) - voff;

		cv::Vec2f m = (s1+s2)/2;
		cv::Vec2f v = s2 - s1;
		float a = atan2(v(1), v(0)) * 180 / CV_PI;

		float len = cv::norm(v);
		float len3 = cv::norm(v2 - v1);

		config[i] = cv::RotatedRect(cv::Point2f(m(0), m(1)), cv::Size(len, 1), a);


		float bestht = -1;
		float bestcost = -1;
		float idealmult;

		//hardcoded
		if(i == ABS || i == CHEST || i == HEAD){
			idealmult = IDEAL_MULT_TORSO;
		}else{
			idealmult = IDEAL_MULT_LIMB;
		}

		for(int h=1;h<config[i].size.width*idealmult;++h){
			config[i].size.height = h;

			float cost = cost2(bestim, config[i], idealmult);
			if(bestcost == -1 || cost < bestcost){
				bestcost = cost;
				bestht = h;
			}
		}
		config[i].size.height = bestht;
		ret[i].radius = bestht/len*len3;
		ret[i].leftOffset = bpLengthsLeft[i];
		ret[i].rightOffset = bpLengthsRight[i];
	}

	return ret;
}



//experimental
cv::Mat expGetCameraMatrix(){
	float param[] = { 0.811468, 1.08224, 0.507597, 0.516744, -0.0212909}; //from camera.cfg

	cv::Mat cameraMatrix;
	cameraMatrix = cv::Mat(4,4,cv::DataType<float>::type,cv::Scalar(0));

	const int WIDTH = 640;
	const int HEIGHT = 480;

	cameraMatrix.at<float>(0,0) = param[0] * WIDTH;
	cameraMatrix.at<float>(1,1) = - param[1] * HEIGHT;
	cameraMatrix.at<float>(0,2) = param[2] * WIDTH - 0.5 - 5; 					  //the +/- 5 is hardcoded to make it closer to the actual toScreen
	cameraMatrix.at<float>(1,2) = - (param[3] * HEIGHT - 0.5) + HEIGHT + 5;		  //the +/- 5 is hardcoded to make it closer to the actual toScreen
	cameraMatrix.at<float>(2,2) = 1;
	return cameraMatrix;
}

cv::Vec2f _toScreen(cv::Vec3f v){
	cv::Mat _v = vec3_to_mat4(v);
	cv::Mat __v = expGetCameraMatrix() * _v;
	return mat4_to_vec2(__v);
}

//todo: make more official
#define NEAR 1 

std::vector<Segment2f> segment3f_to_2f(std::vector<Segment3f> pts, cv::Vec2f offset){
	std::vector<Segment2f> pts2(pts.size());
	for(int i=0;i<pts.size();++i){
		if(pts[i].first(2) < NEAR || pts[i].second(2) < NEAR) return std::vector<Segment2f>(); //opencv bugs out when things are clip thru

		pts2[i].first = toScreen(pts[i].first) - offset;
		pts2[i].second = toScreen(pts[i].second) - offset;
	}
	return pts2;
}


std::vector<cv::Vec2f> vec3f_to_2f(std::vector<cv::Vec3f> pts, cv::Vec2f offset){
	std::vector<cv::Vec2f> pts2(pts.size());
	for(int i=0;i<pts.size();++i){
		if(pts[i](2) < NEAR ) return std::vector<cv::Vec2f>(); //opencv bugs out when things are clip thru

		pts2[i] = toScreen(pts[i]) - offset;
	}
	return pts2;
}

static int countPixels_cyl(cv::Mat im, Cylinder cyl, bool (*cmpFnc)(cv::Vec3b), cv::Vec2f offset){
	std::vector<Segment3f> pts = cylinder_to_segments(cyl.pt1, cyl.pt2, cyl.radius);
	std::vector<Segment2f> pts2 = segment3f_to_2f(pts, offset);

	int cnt = 0;

	PixelPolygon p = polygon_contains_pixels(pts2);
	for(int _x = 0; _x < p.hi.size(); ++_x){
		for(int y = p.lo[_x]; y < p.hi[_x]; ++y){
			int x = _x + p.x_offset;
			
			if(CLAMP_SIZE(x,y,im.size().width,im.size().height))
				if((*cmpFnc)(im.at<cv::Vec3b>(cv::Point(x,y))))
					++cnt;

		}
	}


	return cnt;
}

// (f-b)^2/A
static float cost2_cyl(cv::Mat im, Cylinder cyl, float radmax, cv::Vec2f offset){
	Cylinder ideal = cyl;
	ideal.radius = radmax;
	int idealF = countPixels_cyl(im, ideal, cmpFG, offset);

	int f = countPixels_cyl(im, cyl, cmpFG, offset);
	int b = countPixels_cyl(im, cyl, cmpBG, offset);

	int f_not = idealF - f;

	return std::pow(f_not - b+0.f,2.f);// / rec.size.area();

}

std::vector<Cylinder> projectedCylinderFitting(Skeleton skeletonPositions, CroppedCvMat im, CylinderBody * cylinderBody){
	std::vector<Cylinder> config(NUMLIMBS);

	cv::Mat bestim = im.mat;

	int frame = cylinderBody->bestFrame;
	cv::Vec2f voff = cv::Vec2f(im.offset.x, im.offset.y);
	
	//hardcoded body part lengths
	std::vector<float> bpLengthsLeft(NUMLIMBS, 1);
	std::vector<float> bpLengthsRight(NUMLIMBS, 1);
	
	bpLengthsLeft[HEAD] = 1.5;
	bpLengthsLeft[CHEST] = 1.25;

	bpLengthsRight[LOWERARM_LEFT] = 1.25;
	bpLengthsRight[LOWERARM_RIGHT] = 1.25;
	
	bpLengthsRight[LOWERLEG_LEFT] = 1.25;
	bpLengthsRight[LOWERLEG_RIGHT] = 1.25;


	for(int i=0;i<NUMLIMBS;++i){
		int p1 = getLimbmap()[i].first;
		int p2 = getLimbmap()[i].second;

		cv::Vec3f v1 = mat_to_vec3(skeletonPositions.points.col(p1));
		cv::Vec3f v2 = mat_to_vec3(skeletonPositions.points.col(p2));

		//hardcoded body part lengths
		cv::Vec3f lv1 = v2 + (v1 - v2)*bpLengthsLeft[i];
		cv::Vec3f lv2 = v1 + (v2 - v1)*bpLengthsRight[i];

		cv::Vec2f s1 = toScreen(lv1) - voff;
		cv::Vec2f s2 = toScreen(lv2) - voff;

		cv::Vec2f m = (s1+s2)/2;
		cv::Vec2f v = s2 - s1;
		float a = atan2(v(1), v(0)) * 180 / CV_PI;

		float len = cv::norm(v);
		float len3 = cv::norm(v2 - v1);
		
		config[i] = Cylinder(lv1, lv2, 0.01);


		float bestht = -1;
		float bestcost = -1;
		float idealmult;

		//hardcoded
		if(i == ABS || i == CHEST || i == HEAD){
			idealmult = IDEAL_MULT_TORSO;
		}else{
			idealmult = IDEAL_MULT_LIMB;
		}
		
		float radmax = cv::norm(config[i].pt1 - config[i].pt2) * idealmult / 2;
		//std::cout << i << ": " << radmax << std::endl;

		for(float h=0.01;h<radmax;h+=0.01){
			config[i].radius = h;
		
			float cost = cost2_cyl(bestim, config[i], radmax, voff);
			if(bestcost == -1 || cost < bestcost){
				bestcost = cost;
				bestht = h;
			}
		}
		config[i].radius = bestht;
		config[i].leftOffset = bpLengthsLeft[i];
		config[i].rightOffset = bpLengthsRight[i];
	}

	return config;
}

std::vector<cv::Vec3f> cylinder_to_rectangle(cv::Vec3f a, cv::Vec3f b, float radius){
	

	cv::Vec3f cylaxis = b - a;
	cv::Vec3f originpt = -a;
	cv::Vec3f cross = cylaxis.cross(originpt);

	//draw the rect:
	cv::Vec3f a1 = a + radius * cv::normalize(cross);
	cv::Vec3f a2 = a - radius * cv::normalize(cross);
	cv::Vec3f b1 = b + radius * cv::normalize(cross);
	cv::Vec3f b2 = b - radius * cv::normalize(cross);

	std::vector<cv::Vec3f> ret;
	ret.reserve(4);
	ret.push_back(a1);
	ret.push_back(a2);
	ret.push_back(b2);
	ret.push_back(b1);

	return ret;
}

//makes polygons more "circle-y"
void subdividePolygon(std::vector<Segment3f>& polygon, cv::Vec3f center, float radius){
	
	int j=0;
	while(j<polygon.size()){
		Segment3f segj = polygon[j];
		polygon.erase(polygon.begin()+j);
		auto seg2 = subdivideSegment(segj);
		seg2.first.first = center + radius * cv::normalize(seg2.first.first-center);
		seg2.second.first = center + radius * cv::normalize(seg2.second.first-center);
		seg2.first.second = center + radius * cv::normalize(seg2.first.second-center);
		seg2.second.second = center + radius * cv::normalize(seg2.second.second-center);
		polygon.insert(polygon.begin()+j, seg2.second);
		polygon.insert(polygon.begin()+j, seg2.first);
		j+=2;
	}
}

std::vector<Segment3f> cylinder_to_segments(cv::Vec3f a, cv::Vec3f b, float radius, int numsegments){
	std::vector<Segment3f> ret;

	cv::Vec3f cylaxis = b - a;
	cv::Vec3f originpt = -a;
	cv::Vec3f cross = cv::normalize(cylaxis.cross(originpt));
	cv::Vec3f cross2 = cv::normalize(cylaxis.cross(cross));

	//draw the rect:
	cv::Vec3f a1 = a + radius * (cross);
	cv::Vec3f a2 = a - radius * (cross);
	cv::Vec3f b1 = b + radius * (cross);
	cv::Vec3f b2 = b - radius * (cross);

	cv::Vec3f center_a = (a1+a2)/2;
	cv::Vec3f center_b = (b1+b2)/2;

	ret.push_back(Segment3f(a2,b2));
	ret.push_back(Segment3f(b1,a1));

	int ndiv = std::log(numsegments+0.0)/CV_LOG2;

	//caps

	std::vector<Segment3f> ret_a;
	std::vector<Segment3f> ret_b;

	cv::Vec3f a3 = a + radius * (cross2);
	cv::Vec3f a4 = a - radius * (cross2);
	cv::Vec3f b3 = b + radius * (cross2);
	cv::Vec3f b4 = b - radius * (cross2);
	
	ret_a.push_back(Segment3f(a1,a3));
	ret_a.push_back(Segment3f(a2,a3));
	ret_a.push_back(Segment3f(a1,a4));
	ret_a.push_back(Segment3f(a2,a4));
	ret_b.push_back(Segment3f(b1,b3));
	ret_b.push_back(Segment3f(b2,b3));
	ret_b.push_back(Segment3f(b1,b4));
	ret_b.push_back(Segment3f(b2,b4));

	for(int i=2; i<ndiv; ++i){
		subdividePolygon(ret_a, center_a, radius);
		subdividePolygon(ret_b, center_b, radius);
	}

	for(int i=0;i<ret_a.size();++i){
		ret.push_back(ret_a[i]);
		ret.push_back(ret_b[i]);
	}

	return ret;
}


std::vector<cv::Vec3f> cylinder_to_vertices(cv::Vec3f a, cv::Vec3f b, float radius, int numsegments){
	std::vector<cv::Vec3f> ret;

	cv::Vec3f cylaxis = b - a;
	cv::Vec3f originpt = -a;
	cv::Vec3f cross = cv::normalize(cylaxis.cross(originpt));
	cv::Vec3f cross2 = cv::normalize(cylaxis.cross(cross));

	//draw the rect:
	cv::Vec3f a1 = a + radius * (cross);
	cv::Vec3f a2 = a - radius * (cross);
	cv::Vec3f b1 = b + radius * (cross);
	cv::Vec3f b2 = b - radius * (cross);

	cv::Vec3f center_a = (a1+a2)/2;
	cv::Vec3f center_b = (b1+b2)/2;

	ret.push_back(a1);
	ret.push_back(a2);
	ret.push_back(b1);
	ret.push_back(b2);

	int ndiv = std::log(numsegments+0.0)/CV_LOG2;

	//caps

	cv::Vec3f a3 = a + radius * (cross2);
	cv::Vec3f a4 = a - radius * (cross2);
	cv::Vec3f b3 = b + radius * (cross2);
	cv::Vec3f b4 = b - radius * (cross2);
	
	ret.push_back(a3);
	ret.push_back(a4);
	ret.push_back(b3);
	ret.push_back(b4);

	return ret;
}

float point_on_segment(cv::Vec2f a, cv::Vec2f b, float x, bool * on){
	float londer = (x - a(0))/(b(0)-a(0));
	*on = (londer > 0 && londer < 1);
	return (1-londer)*(a(1)) + (londer)*(b(1));
}

void polygon_contains_pixels(std::vector<cv::Vec2f> points, std::vector<float> * hi, std::vector<float> * lo, float * x_offset){
	cv::Rect bb = cv::boundingRect(points);
	hi->resize(bb.width);
	lo->resize(bb.width);

	for(int c=bb.x;c<bb.x+bb.width;++c){
		float y_hi=bb.y, y_lo=bb.y+bb.height;
		bool on;
		for(int i=0;i<points.size();++i){
			float y = point_on_segment(points[i], points[(i+1)%points.size()], c, &on);
			if(on){
				if(y > y_hi) y_hi = y;
				if(y < y_lo) y_lo = y;
			}
		}
		(*hi)[c-bb.x] = y_hi;
		(*lo)[c-bb.x] = y_lo;
	}

	(*x_offset) = bb.x;
}

PixelPolygon polygon_contains_pixels(std::vector<Segment2f> polygon){
	PixelPolygon p;

	if(polygon.empty()) return p;

	cv::Rect bb = cv::boundingRect(segments_to_points(polygon));
	p.hi.resize(bb.width);
	p.lo.resize(bb.width);
	p.hi_y=-1;
	p.lo_y=-1;

	for(int c=bb.x;c<bb.x+bb.width;++c){
		float y_hi=bb.y, y_lo=bb.y+bb.height;
		bool on;
		for(int i=0;i<polygon.size();++i){
			float y = point_on_segment(polygon[i].first, polygon[i].second, c, &on);
			if(on){
				if(y > y_hi) y_hi = y;
				if(y < y_lo) y_lo = y;
			}
		}
		p.hi[c-bb.x] = y_hi;
		p.lo[c-bb.x] = y_lo;

		if(p.hi_y == -1 || p.hi_y < y_hi) p.hi_y = y_hi;
		if(p.lo_y == -1 || p.lo_y > y_lo) p.lo_y = y_lo;
	}

	(p.x_offset) = bb.x;

	return p;
}


//PixelPolygon polygon_contains_pixels(std::vector<cv::Vec2f> polygon){
//	PixelPolygon p;
//
//	if(polygon.empty()) return p;
//
//	cv::Rect bb = cv::boundingRect(polygon);
//	p.hi.resize(bb.width);
//	p.lo.resize(bb.width);
//	p.hi_y=-1;
//	p.lo_y=-1;
//
//	for(int c=bb.x;c<bb.x+bb.width;++c){
//		float y_hi=bb.y, y_lo=bb.y+bb.height;
//		bool on;
//		for(int i=0;i<polygon.size();++i){
//			float y = point_on_segment(polygon[i].first, polygon[i].second, c, &on);
//			if(on){
//				if(y > y_hi) y_hi = y;
//				if(y < y_lo) y_lo = y;
//			}
//		}
//		p.hi[c-bb.x] = y_hi;
//		p.lo[c-bb.x] = y_lo;
//
//		if(p.hi_y == -1 || p.hi_y < y_hi) p.hi_y = y_hi;
//		if(p.lo_y == -1 || p.lo_y > y_lo) p.lo_y = y_lo;
//	}
//
//	(p.x_offset) = bb.x;
//
//	return p;
//}

// this used to be in loader.cpp


cv::Vec4f findLimbEdges(std::vector<cv::Vec2f> ptVec, int vectorMaxX){

	if(ptVec.size() == 0) return cv::Vec4f();

	int maxX = 0;
	int minX = 10000;

	for(auto it = ptVec.begin(); it != ptVec.end(); ++it){
		if((*it)[0] > maxX){
			maxX = (*it)[0];
		}
		if((*it)[0] < minX){
			minX = (*it)[0];
		}
	}

	//std::cout << "max: " << maxX << " min: " << minX << std::endl;

	std::vector<int> minusVec(maxX - minX + 1, 0);
	std::vector<int> plusVec(maxX - minX + 1, 0);

	int maxY = 0, minY = 0;
	
	for(auto it = ptVec.begin(); it != ptVec.end(); ++it){
		int index = (*it)[0] - minX;
			if(minusVec[index] > (*it)[1]){
				minusVec[index] = (*it)[1];
				if(minY > (*it)[1]) minY = (*it)[1];
			}else if (plusVec[index] < (*it)[1]) {
				plusVec[index] = (*it)[1];
				if(maxY < (*it)[1]) maxY = (*it)[1];
			}
	}

	int maxScore = 0;
	int maxIn = -1;
	int minScore = 0;
	int minIn = -1;

	int error_thresh = 1;


	int offsetLeft, offsetRight;

	for(int i=0; i< -minY; ++i){
		int sc = 0;
		//for(auto it = minusVec.begin(); it != minusVec.end(); ++it){
		int offsetLeft_ = minusVec.size(), offsetRight_ = 0;
		for(int j=0; j<minusVec.size(); ++j){
			auto it = &minusVec[j];
			if( i < -*it + error_thresh) {
				if(offsetLeft_ > j) offsetLeft_ = j;
				offsetRight_ = j;
				++sc;
			}
		}
		sc *= i<maxY?i*1.5:maxY*1;
		if(minScore <= sc) {
			minScore = sc;
			minIn = i;
			offsetLeft = offsetLeft_;
			offsetRight = offsetRight_;
		}
	}

	
	int offsetLeft2, offsetRight2;
	for(int i=0; i< maxY; ++i){
		int sc = 0;
		//for(auto it = plusVec.begin(); it != plusVec.end(); ++it){
		int offsetLeft_ = minusVec.size(), offsetRight_ = 0;
		for(int j=0; j<plusVec.size(); ++j){
			auto it = &plusVec[j];
			if( i < *it + error_thresh){
				if(offsetLeft_ > j) offsetLeft_ = j;
				offsetRight_ = j;
				++sc;
			}
		}
		sc *= i<-minY?i*1.5:-minY*1;
		if(maxScore <= sc) {
			maxScore = sc;
			maxIn = i;
			offsetLeft2 = offsetLeft_;
			offsetRight2 = offsetRight_;
		}
	}

	offsetLeft = offsetLeft < offsetLeft2? offsetLeft + minX: offsetLeft2 + minX;
	offsetRight = offsetRight > offsetRight2? offsetRight + minX - vectorMaxX: offsetRight2 + minX - vectorMaxX;

	//std::cout << "left: " << offsetLeft << " right: " << offsetRight << std::endl;

	return cv::Vec4f(-minIn, maxIn,offsetLeft,offsetRight);
};

float distFromSeg(cv::Vec2f a, cv::Vec2f b, cv::Vec2f pt, int bonusX){

	//std::cout << "a: " << a << " b: "<< b << " pt: " << pt << std::endl;

	if(pt[0] < (a[0]+b[0])/2 - bonusX) pt[0] += bonusX;
	else if(pt[0] > (a[0]+b[0])/2 + bonusX) pt[0] -= bonusX;

	float t = (pt-a).dot(b-a) / (b-a).dot(b-a);
	float currD;

	if(t > 1) currD = (pt-b).dot(pt-b);
	else if(t < 0) currD = (pt-a).dot(pt-a);
	else {
		cv::Vec2f proj = a + t*(b-a);
		currD = (pt-proj).dot(pt-proj);
	}

	//std::cout << "dist: " << currD << std::endl;

	return currD;
}

int dataFnDepth(int p, int l, void* fs){
	FrameSkeletonDepth * frame = (FrameSkeletonDepth*) fs;

	//return the distance from the point to the limb


	if(l==NUMLIMBS) {

		if(frame->frame.at<unsigned short>(p) == 0)
			return 0;
		return 512;
	}

	lmap lm = getLimbmap()[l];

	float ret;

	cv::Vec2f pt = cv::Vec2f(p%frame->frame.cols, p/frame->frame.cols);
	
	int bonusX = 0;

	if(l == CHEST || l == ABS) bonusX = 10;

	float dist = distFromSeg(frame->skeletonPositions2[lm.first], frame->skeletonPositions2[lm.second], pt, bonusX);

	//float dist = distFromSegDepth(frame->skeletonPositions4[lm.first], frame->skeletonPositions4[lm.second], vec4);
	/*if(l == CHEST || l == ABS) dist *= 0.9;
	if(l == UPPERARM_LEFT || l == UPPERARM_RIGHT || l == LOWERARM_LEFT || l == LOWERARM_RIGHT ||
		l == UPPERLEG_LEFT || l == UPPERLEG_RIGHT || l == LOWERLEG_LEFT || l == LOWERLEG_RIGHT ||
		l == FOOT_LEFT || l == FOOT_RIGHT || l == HAND_LEFT || l == HAND_RIGHT ) dist *= 1.15
		;*/
	//ret = dist2Cost(dist);
	ret = dist;
	/*if(dist < 0.2) ret = 0;
	else ret = dist * 2000;*/

	return ret>5000?5000:ret;
}

int getClosestLimb(int pix, FrameSkeletonDepth fs){
	float minDist = 9999;
	int lab;

	for(int i=0;i<=NUMLIMBS;++i){

		float tempD = dataFnDepth(pix, i, (void*)&fs);
		if(minDist > tempD) {
			minDist = tempD;
			lab = i;
		}
	}

	return lab;
}

int smoothFnDepth(int p1, int p2, int l1, int l2, void* fs){
	
	FrameSkeletonDepth * frame = (FrameSkeletonDepth*) fs;
	
	/*if(l1 == l2 && l1 == NUMLIMBS)
	{
		if(frame->frame.at<unsigned short>(p1) == 0 || frame->frame.at<unsigned short>(p2) == 0){
			return 256;
		}else return 512;
	}*/

	if(l1 == l2 && l1 == NUMLIMBS){
		if(frame->frame.at<unsigned short>(p1) == 0 || frame->frame.at<unsigned short>(p2) == 0)
			return 0;
		return 512;
	}

	int retval;
	if(l1 == l2)
	{

		float temp = ((float)(frame->frame.at<unsigned short>(p1)) - (float)(frame->frame.at<unsigned short>(p2)));
		retval = pow(temp, 2);

		//retval = abs(temp);
	}else{
		retval = 512;

	}

	return retval > 512? 512: retval;
}

//todo: make more consistent
//puts variance in varx and vary
void findVariance(std::vector<cv::Vec2f> values, float meanx, float meany, float angle, float * varx, float * vary){
	int M = values.size();
	*varx = 0;
	*vary = 0;

	cv::Mat rot = getRotationMatrix(angle);

	int DECIM = 1;

	//find variance
	for(int i=0;i<M;i+=DECIM){

		cv::Vec2f shift = values[i];
		shift(0) -= meanx;
		shift(1) -= meany;

		//rotation
		cv::Mat _x = rot * cv::Mat(shift);

		float dx = (_x.at<float>(0));
		*varx += (dx * dx)/(M/DECIM);

		float dy = (_x.at<float>(1));
		*vary += (dy * dy)/(M/DECIM);
	}
}

bool buildDepth(Skeleton skeletonPositions, cv::Mat depthIm, CroppedCvMat colorIm, CylinderBody * cb){
	
	/*cv::Vec3b bpColors[NUMLIMBS+1];
	for(int i=0;i<=NUMLIMBS;++i){
		cv::randu(bpColors[i], cv::Scalar(0), cv::Scalar(255));
	}*/

	cv::Vec2f skeletonPositions2[NUMJOINTS];
	for(int bp=0;bp<NUMJOINTS;++bp){
		skeletonPositions2[bp] = toScreen(mat_to_vec3(skeletonPositions.points.col(bp)));
	}

	cv::Vec4f skeletonPositions4[NUMJOINTS];
	for(int bp=0;bp<NUMJOINTS;++bp){
		//skeletonPositions4[bp][0] = skeletonPositions[bp][0];
		//skeletonPositions4[bp][1] = skeletonPositions[bp][1];
		//skeletonPositions4[bp][2] = skeletonPositions[bp][2];
		//skeletonPositions4[bp][3] = 1;

		skeletonPositions4[bp] = skeletonPositions.points.col(bp);
	}

	//cv::Mat greyImg;
	//cv::cvtColor(img, greyImg, CV_BGRA2GRAY);
	//cv::imshow("imag", colorIm);
	//cv::imshow("grey", greyImg);
	//cv::waitKey();

	//trim

		
	int lt = CAPTURE_SIZE_X;
	int rt = 0;
	int up = CAPTURE_SIZE_Y;
	int dn = 0;
	/*
	bool in = false;
		
	for(int x = 0; x < CAPTURE_SIZE_X; ++x){
		for(int y = 0; y < CAPTURE_SIZE_Y; ++y){
			if(colorIm.at<IMGPIXEL>(y,x) != WHITE){
				if( lt > x ) lt = x;
				if( rt < x ) rt = x + 1;
				if( up > y ) up = y;
				if( dn < y ) dn = y + 1;

				in = true;
			}else{
				depthIm.at<unsigned short>(y,x) = 0;
			}
		}
	}

		
	if(!in) return false;
	*/
	lt = 0;
	rt = CAPTURE_SIZE_X;
	up = 0;
	dn = CAPTURE_SIZE_Y;

	cv::Vec2f offset(lt, up);

	cv::Mat img = depthIm(cv::Range(up,dn), cv::Range(lt, rt)).clone();
	//cv::Mat imgC = colorIm(cv::Range(up,dn), cv::Range(lt, rt)).clone();

	//GCoptimizationGridGraph * gc = new GCoptimizationGridGraph(rt - lt, dn - up ,NUMLIMBS+1);


	FrameSkeletonDepth fs;
	fs.frame = img;
	for(int i=0;i<NUMJOINTS;++i){
		fs.skeletonPositions4[i] = skeletonPositions4[i];// - offset;
		fs.skeletonPositions2[i] = skeletonPositions2[i];

	}

	//gc->setDataCost(&dataFnDepth, (void*)&fs);
	//gc->setSmoothCost(&smoothFnDepth, (void*)&fs);

	//cv::Mat imgLab = imgC.clone();

	//cv::Mat armTransform = getTransformMatrix(skeletonPositions2[getLimbmap()[UPPERARM_RIGHT].first], skeletonPositions2[getLimbmap()[UPPERARM_RIGHT].second]);
	//std::vector<cv::Vec2f> ptVec;

	std::vector<cv::Mat> transformMatrices(NUMLIMBS);
	std::vector<cv::Mat> transformMatrices_r(NUMLIMBS);

	for(int j=0;j<getCombineParts().size();++j){
		int i = getCombineParts()[j][0];
		transformMatrices[i] = getTransformMatrix(skeletonPositions2[getLimbmap()[i].first], skeletonPositions2[getLimbmap()[i].second]);
		transformMatrices_r[i] = getTransformMatrix_r(skeletonPositions2[getLimbmap()[i].first], skeletonPositions2[getLimbmap()[i].second]);

	}
	std::vector<std::vector<cv::Vec2f>> ptVectors(NUMLIMBS);
	std::vector<std::vector<cv::Vec2f>> ptVectors_orig(NUMLIMBS);

	//GCoptimization::EnergyType et = gc->expansion(4);

	//std::cout << et << std::endl;

	//cv::Mat bpColorMat = colorIm.mat.clone();

	cb->blobs = cv::Mat(colorIm.mat.rows, colorIm.mat.cols, CV_8UC3, cv::Scalar(255,255,255));
	IMGPIXEL color[NUMLIMBS];
	for(int i=0;i<NUMLIMBS;++i){
		color[i] = IMGPIXEL (rand()%256, rand()%256, rand()%256);
	}
	for(int i=0;i<(rt-lt)*(dn-up);++i){

		//int lab = gc->whatLabel(i);
		int lab = getClosestLimb(i, fs);
				
		if(lab == NUMLIMBS) continue;

		int _x = i%(rt-lt);
		int _y = i/(rt-lt);


		if(getCombinePartsMap()[lab] == -1) continue; 
		cv::Mat tempPix = transformMatrices[getCombinePartsMap()[lab]] * mat_to_homo(cv::Mat(cv::Vec2f(_x, _y)));
		cv::Vec2f final = mat4_to_vec2(tempPix);

		ptVectors[getCombinePartsMap()[lab]].push_back(final);
		ptVectors_orig[getCombinePartsMap()[lab]].push_back(cv::Vec2f(_x,_y));
		
		//bpColorMat.at<cv::Vec3b>(cv::Point2d(_x,_y)-colorIm.offset) = bpColors[lab];
		cv::Point2i bp = cv::Point2i(_x,_y)-colorIm.offset;

		if(bp.x >= 0 && bp.x < cb->blobs.cols && bp.y >= 0 && bp.y < cb->blobs.rows)
			cb->blobs.at<IMGPIXEL>(bp) = color[lab];
	}

	//cv::imshow("color", bpColorMat);
	//cv::imshow("depth", depth2BGR(img));
	//cv::waitKey();
	

			

	for(int j=0;j<getCombineParts().size();++j){
		int i = getCombineParts()[j][0];
		cv::Vec4f edges = findLimbEdges(ptVectors[i],  mat4_to_vec2 (transformMatrices[i] * mat_to_homo( cv::Mat(skeletonPositions2[ getLimbmap()[i].second ]) ))[0]);

#if 1
		cb->partRadii[i] = (-edges[0] + edges[1])/2 /* *
			cv::norm(skeletonPositions4[getLimbmap()[i].first]-skeletonPositions4[getLimbmap()[i].second])*//
			(cv::norm(skeletonPositions2[getLimbmap()[i].first]-skeletonPositions2[getLimbmap()[i].second]) /*+ edges[2] - edges[3]*/);
#else
		cb->partRadii[i] = (-edges[0] + edges[1])/2;
#endif

		cb->leftOffset[i] = edges[2]/cv::norm( skeletonPositions2[getLimbmap()[i].second] - skeletonPositions2[getLimbmap()[i].first]);

		if(cb->leftOffset[i] > 0) cb->leftOffset[i] = 0;
		if(cb->leftOffset[i] < -0.5) cb->leftOffset[i] = -0.5;

		//leftOffset[i] = 0;

		cb->rightOffset[i] = edges[3]/cv::norm( skeletonPositions2[getLimbmap()[i].second] - skeletonPositions2[getLimbmap()[i].first]);

		if(cb->rightOffset[i] < 0) cb->rightOffset[i] = 0;
		if(cb->rightOffset[i] > 0.5) cb->rightOffset[i] = 0.5;

		//rightOffset[i] = 0;

		/*cv::Mat ptb = cv::Mat(skeletonPositions2[getLimbmap()[i].second]);
		cv::Mat tptb = transformMatrices[i] * _2dMat_to_homo(ptb);
		cv::Vec2f tptb_v = _homoMat_to_2dVec(tptb);
		cv::Vec2f ta = skeletonPositions2[getLimbmap()[i].first];*/

		//fixed part radii
		cb->fixedPartRadii[i] = cb->partRadii[i] * (cv::norm(skeletonPositions4[getLimbmap()[i].first]-skeletonPositions4[getLimbmap()[i].second]));

		//limb mean
		cv::Vec2f lm = (skeletonPositions2[getLimbmap()[i].first]+skeletonPositions2[getLimbmap()[i].second])/2;

		//limb vector; 1 -> 2
		cv::Vec2f lv = skeletonPositions2[getLimbmap()[i].second]-skeletonPositions2[getLimbmap()[i].first];

		float varx, vary;
		findVariance(ptVectors_orig[i], lm(0), lm(1), atan2(lv(1), lv(0)), &varx, &vary);
		//findVariance(ptVectors[i], 0, 0, 0, &varx, &vary);

		//find ratio of 2D body part to original 4D body part
		float ratio = (cv::norm(skeletonPositions4[getLimbmap()[i].first]-skeletonPositions4[getLimbmap()[i].second])) / (cv::norm(skeletonPositions2[getLimbmap()[i].first]-skeletonPositions2[getLimbmap()[i].second]));

		cb->varianceX[i] = varx * ratio;
		cb->varianceY[i] = vary * ratio;

	}

	//newskool stuff

	std::vector<BodyPartParam> bpp = rectFitting(skeletonPositions, colorIm, cb);

	std::cout << "rect: \n";

	for(int i=0;i<NUMLIMBS;++i){
		cb->newPartRadii[i] = bpp[i].radius;
		cb->newLeftOffset[i] = bpp[i].leftOffset;
		cb->newRightOffset[i] = bpp[i].rightOffset;

		std::cout << "i " << i << ": " << cb->newPartRadii[i] << std::endl;
	}

	std::vector<Cylinder> cyls = projectedCylinderFitting(skeletonPositions, colorIm, cb);

	std::cout << "cyls: \n";

	for(int i=0;i<NUMLIMBS;++i){
		cb->newPartRadii_cyl[i] = cyls[i].radius;
		cb->newLeftOffset_cyl[i] = cyls[i].leftOffset;
		cb->newRightOffset_cyl[i] = cyls[i].rightOffset;
		
		std::cout << "i " << i << ": " << cb->newPartRadii_cyl[i] << std::endl;
	}

	return true;
};

//pixelpolygon methods

PixelPolygon computeIntersection(const PixelPolygon& a, const PixelPolygon& b){
	PixelPolygon c;

	for(int x=a.x_offset;x<a.hi.size()+a.x_offset;++x){
		if(x >= b.x_offset && x < b.hi.size()+b.x_offset){
			if(c.hi.empty()){
				c.x_offset = x;
			}
			
			float nhi = std::min(a.hi[x-a.x_offset], b.hi[x-b.x_offset]);
			float nlo = std::max(a.lo[x-a.x_offset], b.lo[x-b.x_offset]);

			if(nhi > nlo){
				c.hi.push_back(nhi);
				c.lo.push_back(nlo);

				if(c.hi_y == -1 || c.hi_y < nhi) c.hi_y = nhi;
				if(c.lo_y == -1 || c.lo_y > nlo) c.lo_y = nlo;
			}else{
				c.hi.push_back(-1);
				c.lo.push_back(-1);
			}
		}
	}
	return c;
}