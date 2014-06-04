#include <opencv2\opencv.hpp>
#include <sstream>
#include <iomanip>

int main(){
	int frame = 100, r=0;

	std::cout << "enter directories:\n";

	std::vector<std::string> dirs;
	std::string in;
	while(true){
		std::getline(std::cin, in);
		if(in == "") break;
		else dirs.push_back(in);
	}

	bool play = true;

	while(true){
		bool noValid = true;
		for(auto it = dirs.begin(); it!=dirs.end(); ++it){
			std::stringstream ss;
			ss << *it << "/frame" << std::setfill('0')
				<< std::setw(3) << frame 
				<< "r"
				<< std::setw(2) << r
				<< ".png";
			cv::Mat im = cv::imread(ss.str());

			if(!im.empty()){
				cv::imshow(*it, im);
				noValid = false;
			}
		}
		if(noValid)
			frame = 100;

		if(play){
			++frame;
		}
		char q = cv::waitKey(50);

		switch(q){
		case 'q':
			return 0;
			break;
		case 'a':
			++r;
			if(r>16) r=0;
			break;
		case 'd':
			--r;
			if(r<0) r=16;
			break;
		case 'p':
			play = !play;
		}
	}
}