#include "utils.hpp"
#include "DataModel.hpp"

static std::mutex mtx{};
void print (std::string st) {
	std::unique_lock<std::mutex> lck {mtx};
	std::cout << st;
}

void writePly(string file, vector<ModelFeature> &features){
	ofstream s;
	s.open (file);

	s << "ply" << endl;
	s << "format ascii 1.0 " << endl;
	s << "element vertex " << features.size() << endl;
	s << "property float32 x " << endl;
	s << "property float32 y " << endl;
	s << "property float32 z " << endl;
	s << "property uchar red" << endl;
	s << "property uchar green " << endl;
	s << "property uchar blue " << endl;
	s << "element face 0 " << endl;
	s << "property list uint8 int32 vertex_index { vertex_indices is a list of ints }" << endl;
	s << "end_header " << endl;

	for(auto f : features){
		s << f.position(0) << " " << f.position(1) << " " << f.position(2) << " " <<  (uint16_t)f.color(0) << " " << (uint16_t)f.color(1) << " " << (uint16_t)f.color(2) << endl;
	}

	s.close();
}


void writePly(const std::string &file, const Vec_Points<double> &features){
//	‘__gnu_cxx::__normal_iterator<ModelKeypoint*, std::vector<ModelKeypoint> >
	std::ofstream s {};
	s.open (file);

	s << "ply" << std::endl;
	s << "format ascii 1.0 " << std::endl;
	s << "element vertex " << features.size() << std::endl;
	s << "comment " << file << std::endl;
	s << "property float32 x " << std::endl;
	s << "property float32 y " << std::endl;
	s << "property float32 z " << std::endl;
	s << "end_header " << std::endl;

	for(size_t i {0}; i < features.size(); ++i){
		Points<double> f = features[i];
		s << f[0] << " " << f[1] << " " << f[2] << std::endl;
	}

	s.close();
}



vector<ModelFeature> keypointsToFeatures(vector<ModelKeypoint> *keypoints){
	vector<ModelFeature> ret;
	for(k : *keypoints) ret.push_back(ModelFeature(k.position, RGB888(0,255,0)));
	return ret;
}



Matx13f cross(Matx13f a, Matx13f b){
	auto c = Point3f(a(0), a(1), a(2)).cross(Point3f(b(0), b(1), b(2)));
	return Matx13f(c.x,c.y,c.z);
}
