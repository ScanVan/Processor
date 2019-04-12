#include "utils.hpp"
#include "config.hpp"

static std::mutex mtx{};

void print (std::string st) {
	std::unique_lock<std::mutex> lck {mtx};
	std::cout << st;
}

std::vector<ModelFeature> keypointsToFeatures(std::vector<ModelViewPoint> *keypoints){
	std::vector<ModelFeature> ret;
	for(const auto &k : *keypoints) ret.push_back(ModelFeature(k.position, RGB888(0,255,0)));
	return ret;
}

cv::Matx13f cross(cv::Matx13f a, cv::Matx13f b){
	auto c = cv::Point3f(a(0), a(1), a(2)).cross(cv::Point3f(b(0), b(1), b(2)));
	return cv::Matx13f(c.x,c.y,c.z);
}
