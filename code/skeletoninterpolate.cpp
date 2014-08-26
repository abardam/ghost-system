#include "skeletoninterpolate.h"
#include "definitions.h"
#include "skeletonstatus.h"
#include "cvutil.h"

void interpolate(std::vector<std::vector<char>> &estimRecord, std::vector<Skeleton>& interpPoints){
	if(interpPoints.size() != estimRecord.size()){
		std::cerr << "interpolate error! interpPoints must be the same size as estimRecord\n";
		interpPoints.resize(estimRecord.size());
	}

	for(int j=0;j<NUMJOINTS;++j){
		for(int i=1;i<estimRecord.size();++i){
			if(estimRecord[i][j] == JT_BAD){
				//search for the next JT_GOOD joint
				int nextGood=-1;
				for(int i2=i+1;i2<estimRecord.size();++i2){
					if(estimRecord[i2][j] == JT_BAD_to_GOOD || estimRecord[i2][j] == JT_GOOD){
						nextGood = i2;
						break;
					}
				}
				if(nextGood == -1){
					//reached end of vidRecord; cancel interpolation
					continue;
				}

				float dist = nextGood-i;
				cv::Vec3f interp = mat_to_vec3(interpPoints[nextGood].points.col(j) -
					interpPoints[i-1].points.col(j))/dist;
				cv::Mat base = interpPoints[i-1].points(cv::Rect(j,0,1,3));
				for(int i2=i;i2<nextGood;++i2){
					float dist2 = i2-i;
					cv::Mat add = cv::Mat(dist2*interp);
					interpPoints[i2].points(cv::Rect(j,0,1,3)) = base + add;
				}

				i+=dist;
			}
		}
	}
}