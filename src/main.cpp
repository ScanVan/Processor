//============================================================================
// Name        : Processor.cpp
// Author      : Marcelo E. Kaihara
// Version     :
// Copyright   : 
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <thread>
#include <mutex>
#include <sstream>
#include <iostream>
#include <fstream>
#include <iterator>
#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/core.hpp>

#include "ctpl.hpp"
#include "Queue.hpp"

using namespace std;
using namespace cv;

class PairImages;
class TripletsImages;

ScanVan::thread_safe_queue<PairImages> imgProcQueue {};
ScanVan::thread_safe_queue<TripletsImages> tripletsProcQueue {};
ctpl::thread_pool p(4 /* two threads in the pool */);
std::mutex mtx{};

void print (std::string st) {
	std::unique_lock<std::mutex> lck {mtx};
	std::cout << st;
}

class PairImages {
	int imgNum = 0;
public:
	PairImages() { std::stringstream ss{}; ss << "-->Image pair constructed." << std::endl; print(ss.str()); };
	PairImages(PairImages &n) { imgNum = n.imgNum; std::stringstream ss{}; ss << "-->Copy constructor. Image pair " << imgNum << " constructed." << std::endl; print(ss.str()); };
	PairImages(PairImages &&n) { imgNum = n.imgNum; std::stringstream ss{}; ss << "-->Move constructor. Image pair " << imgNum << " constructed." << std::endl; print(ss.str()); };
	PairImages(int &n) { imgNum = n; std::stringstream ss{}; ss << "-->Image pair " << imgNum << " constructed." << std::endl; print(ss.str()); };
	void setImgNum (int n) { imgNum = n; }
	int getImgNum() const { return imgNum; }
	~PairImages() {  std::stringstream ss{}; ss << "-->Image pair " << imgNum << " destructed." << std::endl; print(ss.str()); };
	PairImages & operator= (PairImages &p) { imgNum = p.imgNum; std::stringstream ss{}; ss << "Image pair " << imgNum << " copied." << std::endl; print(ss.str()); return *this;};
};

class PairListPoints {
	int idx1 = 0;
	int idx2 = 1;
public:
	PairListPoints() { std::stringstream ss{}; ss << "-->Pair of list of points constructed." << std::endl; print(ss.str()); };
	PairListPoints(int n1, int n2) : idx1 {n1}, idx2{n2} { std::stringstream ss{}; ss << "-->Pair of list of points " << idx1 << ", " << idx2 << " constructed." << std::endl; print(ss.str()); };
	PairListPoints(PairListPoints &n) { idx1 = n.idx1; idx2 = n.idx2; std::stringstream ss{}; ss << "-->Copy constructor. Pair of list of points " << idx1 << ", " << idx2 << " constructed." << std::endl; print(ss.str()); };
	PairListPoints(PairListPoints &&n) { idx1 = n.idx1; idx2 = n.idx2; std::stringstream ss{}; ss << "-->Move constructor. Pair of list of points " << idx1 << ", " << idx2 << " constructed." << std::endl; print(ss.str()); };
	void setListIdx (int a, int b) { idx1 = a; idx2 = b; }
	int getListIdx1 () const { return idx1; }
	int getListIdx2 () const { return idx2; }
	PairListPoints & operator=(const PairListPoints &n) {idx1 = n.idx1; idx2 = n.idx2; std::stringstream ss{}; ss << "-->Assignment operator. Pair of list of points " << idx1 << ", " << idx2 << " constructed." << std::endl; print(ss.str()); return *this;};
	~PairListPoints() { std::stringstream ss{}; ss << "-->Pair of list of points " << idx1 << ", " << idx2 << " destructed." << std::endl; print(ss.str()); };
};

class Triplets {
	int idx1 = 0;
	int idx2 = 1;
	int idx3 = 2;
	// List of points
public:
	Triplets() { std::stringstream ss{}; ss << "-->Triplets constructed." << std::endl; print(ss.str()); };
	Triplets(int n1, int n2, int n3) : idx1 {n1}, idx2{n2}, idx3{n3} { std::stringstream ss{}; ss << "-->Triplets " << idx1 << ", " << idx2 << ", " << idx3 << " constructed." << std::endl; print(ss.str()); };
	Triplets(Triplets &n) { idx1 = n.idx1; idx2 = n.idx2; idx3 = n.idx3; std::stringstream ss{}; ss << "-->Copy constructor. Triplets " << idx1 << ", " << idx2 << ", " << idx3 << " constructed." << std::endl; print(ss.str()); };
	Triplets(Triplets &&n) { idx1 = n.idx1; idx2 = n.idx2; idx3 = n.idx3; std::stringstream ss{}; ss << "-->Move constructor. Triplets " << idx1 << ", " << idx2 << ", " << idx3 << " constructed." << std::endl; print(ss.str()); };
	void setListIdx (int a, int b, int c) { idx1 = a; idx2 = b; idx3 = c;}
	int getListIdx1 () const { return idx1; }
	int getListIdx2 () const { return idx2; }
	int getListIdx3 () const { return idx3; }
	Triplets & operator= (Triplets &n) { idx1 = n.idx1; idx2 = n.idx2; idx3 = n.idx3; std::stringstream ss{}; ss << "-->Assignment operator. Triplets " << idx1 << ", " << idx2 << ", " << idx3 << " constructed." << std::endl; print(ss.str()); return *this;};
	~Triplets() { std::stringstream ss{}; ss << "-->Triplets " << idx1 << ", " << idx2 << ", " << idx3 << " destructed." << std::endl; print(ss.str()); };
};

class TripletsImages {
	int idx1 = 0;
	int idx2 = 1;
	int idx3 = 2;
	std::shared_ptr<PairImages> img1 {};
	std::shared_ptr<PairImages> img2 {};
	std::shared_ptr<PairImages> img3 {};
	// List of points
public:
	TripletsImages() { std::stringstream ss{}; ss << "-->TripletsImages constructed." << std::endl; print(ss.str()); };
	TripletsImages(int n1, int n2, int n3, std::shared_ptr<PairImages> im1, std::shared_ptr<PairImages> im2, std::shared_ptr<PairImages> im3) :
			idx1 { n1 }, idx2 { n2 }, idx3 { n3 }, img1 {im1}, img2 {im2}, img3 {im3}  {
		std::stringstream ss { };
		ss << "-->TripletsImages " << idx1 << ", " << idx2 << ", " << idx3 << " constructed." << std::endl;
		print(ss.str());
	}
	TripletsImages(TripletsImages &n) {
		idx1 = n.idx1;
		idx2 = n.idx2;
		idx3 = n.idx3;
		img1 = n.img1;
		img2 = n.img2;
		img3 = n.img3;
		std::stringstream ss { };
		ss << "-->Copy constructor. TripletsImages " << idx1 << ", " << idx2 << ", " << idx3 << " constructed." << std::endl;
		print(ss.str());
	}
	TripletsImages(TripletsImages &&n) {
		idx1 = n.idx1;
		idx2 = n.idx2;
		idx3 = n.idx3;
		img1 = n.img1;
		img2 = n.img2;
		img3 = n.img3;
		std::stringstream ss { };
		ss << "-->Move constructor. TripletsImages " << idx1 << ", " << idx2 << ", " << idx3 << " constructed." << std::endl;
		print(ss.str());
	}
	void setListIdx (int a, int b, int c) { idx1 = a; idx2 = b; idx3 = c;}
	void setSharedPtr(std::shared_ptr<PairImages> a, std::shared_ptr<PairImages> b, std::shared_ptr<PairImages> c) {
		img1 = a;
		img2 = b;
		img3 = c;
	}
	int getListIdx1 () const { return idx1; }
	int getListIdx2 () const { return idx2; }
	int getListIdx3 () const { return idx3; }
	TripletsImages & operator=(TripletsImages &n) {
		idx1 = n.idx1;
		idx2 = n.idx2;
		idx3 = n.idx3;
		img1 = n.img1;
		img2 = n.img2;
		img3 = n.img3;
		std::stringstream ss { };
		ss << "-->Assignment operator. TripletsImages " << idx1 << ", " << idx2 << ", " << idx3 << " constructed." << std::endl;
		print(ss.str());
		return *this;
	}
	~TripletsImages() {
		std::stringstream ss { };
		ss << "-->TripletsImages " << idx1 << ", " << idx2 << ", " << idx3 << " destructed." << std::endl;
		print(ss.str());
	}

};




typedef Vec<uint8_t, 3> RGB888;

class ModelFeature{
public:
	Matx13f position;
	RGB888 color;
	ModelFeature(){}
	ModelFeature(Matx13f position) : position(position){}
	ModelFeature(Matx13f position, RGB888 color) : position(position), color(color){}
};

//Represent
class ModelKeypoint{
public:
	Matx13f position;
	//...

	ModelKeypoint(){}
	ModelKeypoint(Matx13f position) : position(position){}
};



//void writePly(string file, vector<Matx13f> points){
void writePly(string file, vector<ModelFeature> features){
//	‘__gnu_cxx::__normal_iterator<ModelKeypoint*, std::vector<ModelKeypoint> >
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

/*
vector<Matx13f> featuresToPoints(vector<ModelFeature> *features){
	vector<Matx13f> ret;
	for(f : *features) ret.push_back(f.position);
	return ret;
}*/

vector<ModelFeature> keypointsToFeatures(vector<ModelKeypoint> *keypoints){
	vector<ModelFeature> ret;
	for(k : *keypoints) ret.push_back(ModelFeature(k.position, RGB888(0,255,0)));
	return ret;
}



class Model {
	// change to a list of indices
	int idx1 = 0;
	int idx2 = 1;
	int idx3 = 2;
	// List of points for models

public:
	vector<ModelKeypoint> keypoints;
	vector<ModelFeature> features;



	Model() {
		std::stringstream ss { };
		ss << "-->Model constructed." << std::endl;
		print(ss.str());
	}
	Model(int n1, int n2, int n3) :
			idx1 { n1 }, idx2 { n2 }, idx3 { n3 } {
		std::stringstream ss { };
		ss << "-->Model " << idx1 << ", " << idx2 << ", " << idx3 << " constructed." << std::endl;
		print(ss.str());
	}
	Model(Model &n) {
		idx1 = n.idx1;
		idx2 = n.idx2;
		idx3 = n.idx3;
		std::stringstream ss { };
		ss << "-->Copy constructor. Model " << idx1 << ", " << idx2 << ", " << idx3 << " constructed." << std::endl;
		print(ss.str());
	}
	Model(Model &&n) {
		idx1 = n.idx1;
		idx2 = n.idx2;
		idx3 = n.idx3;
		std::stringstream ss { };
		ss << "-->Move constructor. Model " << idx1 << ", " << idx2 << ", " << idx3 << " constructed." << std::endl;
		print(ss.str());
	}


	void setListIdx(int a, int b, int c) {
		idx1 = a;
		idx2 = b;
		idx3 = c;
	}
	int getListIdx1() const {
		return idx1;
	}
	int getListIdx2() const {
		return idx2;
	}
	int getListIdx3() const {
		return idx3;
	}
	Model & operator=(Model &n) {
		idx1 = n.idx1;
		idx2 = n.idx2;
		idx3 = n.idx3;
		std::stringstream ss { };
		ss << "-->Assignment operator. Model " << idx1 << ", " << idx2 << ", " << idx3 << " constructed." << std::endl;
		print(ss.str());
		return *this;
	}
	Model & operator=(Model &&n) {
		idx1 = n.idx1;
		idx2 = n.idx2;
		idx3 = n.idx3;
		std::stringstream ss { };
		ss << "-->Move assignment operator. Model " << idx1 << ", " << idx2 << ", " << idx3 << " constructed." << std::endl;
		print(ss.str());
		return *this;
	}

	~Model() {
		std::stringstream ss { };
		ss << "-->Model " << idx1 << ", " << idx2 << ", " << idx3 << " destructed." << std::endl;
		print(ss.str());
	}
};

//=========================================================================================================

void GeneratePairImages () {

	int i {0};

	for (;;) {
		i++;
		PairImages p1{i};
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		imgProcQueue.push(p1);
		std::stringstream ss {};
		ss << "=========================" << std::endl << "Send pair images " << p1.getImgNum() << std::endl << "=========================" << std::endl;
		print (ss.str());
	}
}

PairListPoints FeatureExtraction (std::shared_ptr<PairImages> im1, std::shared_ptr<PairImages> im2) {

	std::stringstream ss {};
	ss << "=========================" << std::endl << "Feature Extraction " << im1->getImgNum() << " - "  << im2->getImgNum() << std::endl << "=========================" << std::endl;
	print (ss.str());

	PairListPoints p1 { im1->getImgNum(), im2->getImgNum() };
	return p1;
}

Triplets CommonPointsComputation (PairListPoints &p1, PairListPoints &p2) {

	std::stringstream ss {};
	ss << "=========================" << std::endl << "Common Points Computation (" << p1.getListIdx1() << " - "  << p1.getListIdx2() << ") (" << p2.getListIdx1() << " - " << p2.getListIdx2() << ")" <<  std::endl << "=========================" << std::endl;
	print (ss.str());

	if (p1.getListIdx2() != p2.getListIdx1()) {
		throw std::runtime_error ("Error in indexes in CommonPointComputation");
	}

	Triplets t1 { p1.getListIdx1(), p1.getListIdx2(), p2.getListIdx2()};

	return t1;
}

void ProcFeatures() {

	std::vector<std::shared_ptr<PairImages>> v{};
	std::vector<std::shared_ptr<PairImages>> ims{};
	std::vector<PairListPoints> lp{};

	for (;;) {
		std::shared_ptr<PairImages> receivedPairImages { };
		receivedPairImages = imgProcQueue.wait_pop();
		std::stringstream ss {};
		ss << "=========================" << std::endl << "Received pair images " << receivedPairImages->getImgNum() << std::endl << "=========================" << std::endl;
		print (ss.str());

		v.push_back(receivedPairImages);
		ims.push_back(receivedPairImages);

		if (v.size() == 2) {
			lp.push_back(FeatureExtraction (v[0], v[1]));
			v[0]=v[1];
			v.pop_back();
		}

		if (lp.size() == 2) {
			Triplets p1 {CommonPointsComputation (lp[0], lp[1])};
			lp[0] = lp[1];
			lp.pop_back();
			TripletsImages ti { p1.getListIdx1(), p1.getListIdx2(), p1.getListIdx3(), ims[0], ims[1], ims[2] };
			for (auto it = ims.begin(); it!= ims.end()-1; ++it) {
				*it = *(it+1);
			}
			ims.pop_back();
			tripletsProcQueue.push(ti);
		}
	}
}

Model PoseEstimation (Triplets &t1) {
	std::stringstream ss { };
	ss << "=========================" << std::endl << "Pose Estimation with triplets" << t1.getListIdx1() << " - " << t1.getListIdx2() << " - " << t1.getListIdx3() << std::endl
			<< "=========================" << std::endl;
	print(ss.str());

	Model m1 { t1.getListIdx1(), t1.getListIdx2(), t1.getListIdx3() };
	return m1;
}


Matx13f cross(Matx13f a, Matx13f b){
	auto c = Point3f(a(0), a(1), a(2)).cross(Point3f(b(0), b(1), b(2)));
	return Matx13f(c.x,c.y,c.z);
}

void FusionModel (Model *m1, Model *m2) {
	std::stringstream ss { };
	ss << "=========================" << std::endl << "Fusion Model (" << m1->getListIdx1() << " - " << m1->getListIdx2() << " - " << m1->getListIdx3() << ")"
			<< std::endl
			<< "=========================" << std::endl;
	print(ss.str());

	assert(m1->keypoints.size() >= 3);
	assert(m2->keypoints.size() == 3);
	auto m1CapturesSize = m1->keypoints.size();
	auto m1C1 = m1->keypoints[m1CapturesSize-2].position;
	auto m1C2 = m1->keypoints[m1CapturesSize-1].position;
	auto m2C1 = m2->keypoints[0].position;
	auto m2C2 = m2->keypoints[1].position;
	auto m2C3 = m2->keypoints[2].position;
	auto scaleFactor = norm(m1C2-m1C1)/norm(m2C2-m2C1);
	auto t1 = (1/norm(m1C2-m1C1)) * (m1C2-m1C1);
	auto t2 = (1/norm(m2C2-m2C1)) * (m2C2-m2C1);
	auto v = cross(t1,t2);
	auto c = t1.dot(t2);
	auto w = Matx33f(0.0,-v(2),v(1),v(2),0.0,-v(0),-v(1),v(0),0.0);
	auto rotation = Matx33f().eye() + w + (1/(1+c))*(w*w);
	auto translation = m1C1-m2C1;

	for(auto source : m2->features){
		auto feature = ModelFeature();
		feature.color = source.color;
		feature.position = translation + scaleFactor * (source.position* rotation);
		m1->features.push_back(feature);
	}
	ModelKeypoint m1C3;
	m1C3.position = translation + scaleFactor * (m2C3*rotation);
	m1->keypoints.push_back(m1C3);
}

void ProcPose() {

	std::vector <Model> vecm {};
	Model m {};


	for (;;) {
		std::shared_ptr<TripletsImages> receivedTripletsImages { };
		receivedTripletsImages = tripletsProcQueue.wait_pop();
		std::stringstream ss {};
		ss << "=========================" << std::endl << "Received triplets images " << receivedTripletsImages->getListIdx1() << ", " << receivedTripletsImages->getListIdx2() << ", " << receivedTripletsImages->getListIdx3() << std::endl << "=========================" << std::endl;
		print (ss.str());

		Triplets t1 {receivedTripletsImages->getListIdx1(), receivedTripletsImages->getListIdx2(), receivedTripletsImages->getListIdx3()};
		Model m1 {PoseEstimation (t1)};

		FusionModel (&m1, &m);
	}
}


int main() {


	std::thread GenPairs (GeneratePairImages);
	std::thread ProcessFeatureExtraction (ProcFeatures);
	std::thread ProcessPoseEstimation (ProcPose);

	GenPairs.join();
	ProcessFeatureExtraction.join();
	ProcessPoseEstimation.join();

	return 0;

}


#include <iostream>
#include <fstream>
void testModelFusion(){
	vector<Matx13f> chopper;
	 std::fstream myfile("/home/scanvan/scanvan/ply/chopper.cp", std::ios_base::in);

	float x,y,z;
	while (myfile >> x >> y >> z)
	{
		chopper.push_back(Matx13f(x,y,z));
	}

	Model mainModel;
	mainModel.keypoints.push_back(ModelKeypoint(Matx13f(0,0,0)));
	mainModel.keypoints.push_back(ModelKeypoint(Matx13f(100,0,0)));
	mainModel.keypoints.push_back(ModelKeypoint(Matx13f(200,0,0)));
	mainModel.keypoints.push_back(ModelKeypoint(Matx13f(300,0,0)));
	for(auto p : chopper)mainModel.features.push_back(ModelFeature(p));

	for(int i = 0;i < 10;i ++){
		Model triplet;
		triplet.keypoints.push_back(ModelKeypoint(Matx13f(0,0,0)));
		triplet.keypoints.push_back(ModelKeypoint(Matx13f(100 - i*2,10,5)));
		triplet.keypoints.push_back(ModelKeypoint(Matx13f(200,30,i*10)));

		for(auto p : chopper)triplet.features.push_back(ModelFeature(p, RGB888(255,0, 25*i)));
		FusionModel(&mainModel, &triplet);

	}
	writePly("keypoints.ply",  keypointsToFeatures(&mainModel.keypoints));
	writePly("features.ply",  mainModel.features);


	//	Model mainModel;
	//	mainModel.keypoints.push_back(ModelKeypoint(Matx13f(0,0,0)));
	//	mainModel.keypoints.push_back(ModelKeypoint(Matx13f(10,1,9)));
	//	mainModel.keypoints.push_back(ModelKeypoint(Matx13f(16,7,4)));
	//	mainModel.keypoints.push_back(ModelKeypoint(Matx13f(22,11,5)));
	//
	//	Model triplet;
	//	triplet.keypoints.push_back(ModelKeypoint(Matx13f(0,0,0)));
	//	triplet.keypoints.push_back(ModelKeypoint(Matx13f(2,7,5)));
	//	triplet.keypoints.push_back(ModelKeypoint(Matx13f(1,8,4)));
	//	pos_1 = [[0,0,0],[10,1,9],[20,7,4],[30,11,5]]
	//	pos_2 = [[0,0,0],[2,7,5],[20,20,0]]

}





